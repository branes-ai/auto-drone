"""
Detection Service

Provides access to object detections from multiple cameras.
"""

import asyncio
import time
from threading import Lock
from typing import Optional, Dict, List, TYPE_CHECKING

import zenoh

# Import data types (in sim_interfaces/airsim_zenoh_bridge/)
try:
    from data_types import DetectionList, CameraDetectionList, Detection
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / "sim_interfaces" / "airsim_zenoh_bridge"))
    from data_types import DetectionList, CameraDetectionList, Detection

if TYPE_CHECKING:
    from ..state import MissionState


# Camera geometry from robot_quadrotor_vision.jsonc
CAMERA_GEOMETRY = {
    "front": {
        "position": (0.30, 0.0, -0.05),
        "orientation": (0.0, -15.0, 0.0),  # roll, pitch, yaw in degrees
        "fov": 90.0,
    },
    "back": {
        "position": (-0.15, 0.0, 0.0),
        "orientation": (0.0, 0.0, 180.0),
        "fov": 90.0,
    },
    "down": {
        "position": (0.0, 0.0, 0.05),
        "orientation": (0.0, -90.0, 0.0),
        "fov": 90.0,
    },
    "chase": {
        "position": (-10.0, 0.0, -1.0),
        "orientation": (0.0, -11.46, 0.0),
        "fov": 90.0,
    },
}


class DetectionService:
    """
    Service for detection access.

    Handles:
    - Subscribing to per-camera detection topics
    - Providing thread-safe detection access
    - Camera geometry for world position estimation

    Detections are published by external detector nodes (e.g., yolo_multi_camera_detector.py).
    """

    def __init__(
        self,
        session: zenoh.Session,
        state: 'MissionState',
        robot_id: str = "drone",
        cameras: List[str] = None
    ):
        """
        Initialize detection service.

        Args:
            session: Zenoh session
            state: Mission state
            robot_id: Robot identifier for topics
            cameras: List of camera IDs to subscribe to
        """
        self.session = session
        self.state = state
        self.robot_id = robot_id
        self.cameras = cameras or ["front"]

        # Per-camera detection state
        self._detections: Dict[str, Optional[CameraDetectionList]] = {}
        self._locks: Dict[str, Lock] = {}
        self._timestamps: Dict[str, float] = {}
        for cam in self.cameras:
            self._detections[cam] = None
            self._locks[cam] = Lock()
            self._timestamps[cam] = 0.0

        # Subscribers
        self._subs = []

        # Statistics
        self.detection_counts: Dict[str, int] = {cam: 0 for cam in self.cameras}

    async def start(self) -> None:
        """Start the service - subscribe to detection topics."""
        for camera_id in self.cameras:
            topic = f"robot/{self.robot_id}/perception/detections/{camera_id}"
            sub = self.session.declare_subscriber(
                topic,
                lambda sample, cid=camera_id: self._on_detection(sample, cid)
            )
            self._subs.append(sub)
            print(f"  DetectionService: subscribed to {topic}")

    async def stop(self) -> None:
        """Stop the service."""
        pass

    def _on_detection(self, sample: zenoh.Sample, camera_id: str) -> None:
        """Handle detection from a specific camera."""
        try:
            # Try CameraDetectionList first (includes camera metadata)
            detections = CameraDetectionList.deserialize(bytes(sample.payload))
            with self._locks[camera_id]:
                self._detections[camera_id] = detections
                self._timestamps[camera_id] = time.time()
                self.detection_counts[camera_id] += 1
        except Exception:
            try:
                # Fall back to DetectionList (legacy format)
                detections = DetectionList.deserialize(bytes(sample.payload))
                cam_geom = self.get_camera_geometry(camera_id)
                cam_det = CameraDetectionList(
                    timestamp_us=detections.timestamp_us,
                    frame_id=detections.frame_id,
                    image_width=detections.image_width,
                    image_height=detections.image_height,
                    inference_time_ms=detections.inference_time_ms,
                    camera_id=camera_id,
                    camera_position=cam_geom["position"],
                    camera_orientation=cam_geom["orientation"],
                    detections=detections.detections
                )
                with self._locks[camera_id]:
                    self._detections[camera_id] = cam_det
                    self._timestamps[camera_id] = time.time()
                    self.detection_counts[camera_id] += 1
            except Exception:
                pass

    def get_detections(self, camera_id: str, max_age: float = 0.5) -> Optional[CameraDetectionList]:
        """
        Get latest detections from a specific camera.

        Args:
            camera_id: Camera identifier
            max_age: Maximum age in seconds (older detections return None)

        Returns:
            CameraDetectionList or None if no recent detections
        """
        if camera_id not in self._locks:
            return None
        with self._locks[camera_id]:
            if self._detections[camera_id] is None:
                return None
            if time.time() - self._timestamps[camera_id] > max_age:
                return None
            return self._detections[camera_id]

    def get_all_detections(self, max_age: float = 0.5) -> Dict[str, CameraDetectionList]:
        """
        Get latest detections from all cameras.

        Args:
            max_age: Maximum age in seconds

        Returns:
            Dict mapping camera_id to CameraDetectionList
        """
        result = {}
        for camera_id in self.cameras:
            det = self.get_detections(camera_id, max_age)
            if det:
                result[camera_id] = det
        return result

    def get_target_detections(
        self,
        target_class: str,
        max_age: float = 0.5
    ) -> Dict[str, List[Detection]]:
        """
        Get detections of a specific target class from all cameras.

        Args:
            target_class: Target class name to filter
            max_age: Maximum age in seconds

        Returns:
            Dict mapping camera_id to list of matching detections
        """
        result = {}
        for camera_id in self.cameras:
            det_list = self.get_detections(camera_id, max_age)
            if det_list:
                matches = det_list.get_by_class(target_class)
                if matches:
                    result[camera_id] = matches
        return result

    def get_best_detection(
        self,
        target_class: str,
        max_age: float = 0.5
    ) -> tuple:
        """
        Get best detection of target class across all cameras.

        Args:
            target_class: Target class name
            max_age: Maximum age in seconds

        Returns:
            Tuple of (camera_id, Detection) or (None, None) if not found
        """
        best_camera = None
        best_detection = None
        best_confidence = 0.0

        for camera_id in self.cameras:
            det_list = self.get_detections(camera_id, max_age)
            if det_list:
                det = det_list.get_best(target_class)
                if det and det.confidence > best_confidence:
                    best_camera = camera_id
                    best_detection = det
                    best_confidence = det.confidence

        return (best_camera, best_detection)

    def get_camera_geometry(self, camera_id: str) -> dict:
        """
        Get camera geometry for world position estimation.

        Args:
            camera_id: Camera identifier

        Returns:
            Dict with 'position', 'orientation', 'fov' keys
        """
        return CAMERA_GEOMETRY.get(camera_id, CAMERA_GEOMETRY["front"])

    def has_detections(self, camera_id: Optional[str] = None, max_age: float = 0.5) -> bool:
        """
        Check if any detections are available.

        Args:
            camera_id: Specific camera to check, or None for any camera
            max_age: Maximum age in seconds

        Returns:
            True if recent detections exist
        """
        if camera_id:
            return self.get_detections(camera_id, max_age) is not None
        return len(self.get_all_detections(max_age)) > 0

    async def wait_for_detections(self, timeout: float = 10.0) -> bool:
        """
        Wait for at least one camera to receive detections.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if detections received, False if timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            if self.has_detections():
                return True
            await asyncio.sleep(0.1)
        return False
