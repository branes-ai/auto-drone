"""
Mission State Management

Provides thread-safe state containers for mission execution.
"""

import math
import time
from dataclasses import dataclass, field
from threading import Lock
from typing import Optional, List, Dict, Any

# Import Odometry from data_types (in sim_interfaces/airsim_zenoh_bridge/)
# Ensure PYTHONPATH includes that directory, or run from project root
try:
    from data_types import Odometry
except ImportError:
    # Fallback: try relative import for when running from airsim_zenoh_bridge
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent.parent / "sim_interfaces" / "airsim_zenoh_bridge"))
    from data_types import Odometry


@dataclass
class TrackedObject:
    """
    An object detected during scan with estimated world position.

    Consolidates duplicated TrackedObject/DetectedObject from mission scripts.
    """
    class_name: str
    confidence: float
    bearing_x: float      # -1 to +1 (left to right in camera frame)
    bearing_y: float      # -1 to +1 (top to bottom in camera frame)
    area: int             # Bounding box area in pixels
    camera_id: str        # Which camera detected this
    drone_x: float        # Drone position at detection time
    drone_y: float        # Drone position at detection time
    drone_z: float        # Drone position at detection time (NED, negative = up)
    drone_yaw: float      # Drone yaw at detection time (radians)
    timestamp: float      # Detection timestamp
    world_x: float = 0.0  # Estimated world X position
    world_y: float = 0.0  # Estimated world Y position

    def estimate_world_position(
        self,
        camera_fov_deg: float = 90.0,
        camera_pitch_deg: float = -15.0,
        camera_yaw_deg: float = 0.0
    ) -> None:
        """
        Estimate world position from bearing and drone pose.

        Args:
            camera_fov_deg: Camera field of view in degrees
            camera_pitch_deg: Camera pitch relative to drone (negative = looking down)
            camera_yaw_deg: Camera yaw relative to drone (180 = rear camera)
        """
        # Estimate distance from object size (larger = closer)
        estimated_distance = max(10.0, 100.0 / math.sqrt(self.area / 1000.0 + 1))

        camera_pitch_rad = math.radians(camera_pitch_deg)
        camera_yaw_rad = math.radians(camera_yaw_deg)
        half_fov = math.radians(camera_fov_deg / 2)

        # Compute angles in world frame
        horizontal_angle = self.bearing_x * half_fov + camera_yaw_rad
        vertical_angle = self.bearing_y * half_fov + camera_pitch_rad

        target_heading = self.drone_yaw + horizontal_angle
        horizontal_dist = estimated_distance * math.cos(vertical_angle)

        self.world_x = self.drone_x + horizontal_dist * math.cos(target_heading)
        self.world_y = self.drone_y + horizontal_dist * math.sin(target_heading)


class ObjectInventory:
    """
    Catalog of detected objects from scan.

    Thread-safe collection that merges nearby detections of the same class.
    """

    def __init__(self, merge_distance: float = 10.0):
        """
        Args:
            merge_distance: Objects within this distance are merged (meters)
        """
        self.merge_distance = merge_distance
        self.objects: List[TrackedObject] = []
        self._lock = Lock()

    def add(self, obj: TrackedObject) -> None:
        """
        Add object, merging with nearby existing detections.

        If an object of the same class exists within merge_distance,
        update it if the new detection has higher confidence.
        """
        with self._lock:
            for existing in self.objects:
                if existing.class_name.lower() != obj.class_name.lower():
                    continue
                dx = obj.world_x - existing.world_x
                dy = obj.world_y - existing.world_y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < self.merge_distance:
                    # Merge: keep higher confidence detection
                    if obj.confidence > existing.confidence:
                        existing.world_x = obj.world_x
                        existing.world_y = obj.world_y
                        existing.confidence = obj.confidence
                        existing.area = obj.area
                        existing.camera_id = obj.camera_id
                        existing.timestamp = obj.timestamp
                    return
            # No nearby match, add as new object
            self.objects.append(obj)

    def get_best(self, class_name: str) -> Optional[TrackedObject]:
        """Get highest confidence object of given class."""
        with self._lock:
            matches = [
                o for o in self.objects
                if o.class_name.lower() == class_name.lower()
            ]
            return max(matches, key=lambda o: o.confidence) if matches else None

    def get_largest(self, class_name: str) -> Optional[TrackedObject]:
        """Get largest (by area) object of given class."""
        with self._lock:
            matches = [
                o for o in self.objects
                if o.class_name.lower() == class_name.lower()
            ]
            return max(matches, key=lambda o: o.area) if matches else None

    def get_all(self, class_name: Optional[str] = None) -> List[TrackedObject]:
        """Get all objects, optionally filtered by class."""
        with self._lock:
            if class_name:
                return [
                    o for o in self.objects
                    if o.class_name.lower() == class_name.lower()
                ]
            return list(self.objects)

    def count(self, class_name: Optional[str] = None) -> int:
        """Count objects, optionally filtered by class."""
        with self._lock:
            if class_name:
                return sum(
                    1 for o in self.objects
                    if o.class_name.lower() == class_name.lower()
                )
            return len(self.objects)

    def clear(self) -> None:
        """Clear all objects from inventory."""
        with self._lock:
            self.objects.clear()

    def print_inventory(self) -> None:
        """Print inventory contents."""
        with self._lock:
            if not self.objects:
                print("  Inventory: empty")
                return
            print(f"  Inventory: {len(self.objects)} objects")
            for i, obj in enumerate(self.objects):
                print(f"    [{i}] {obj.class_name}: conf={obj.confidence:.2f} "
                      f"pos=({obj.world_x:.1f}, {obj.world_y:.1f}) "
                      f"camera={obj.camera_id}")


class MissionState:
    """
    Shared mission state accessible by all phases.

    Thread-safe container for odometry, detections, selected targets, etc.
    Phases read and write to this shared state during execution.
    """

    def __init__(self):
        # Odometry (updated by DroneService)
        self._odom: Optional[Odometry] = None
        self._odom_lock = Lock()

        # Detection inventory (populated during scan phase)
        self.inventory = ObjectInventory()

        # Selected target (set by SelectPhase, used by Navigate/Approach)
        self._selected_target: Optional[TrackedObject] = None
        self._target_lock = Lock()

        # Mission parameters (set from config, read by phases)
        self.params: Dict[str, Any] = {}

        # Phase-specific data storage (for inter-phase communication)
        self.phase_data: Dict[str, Any] = {}

        # Mission running flag
        self._running = True
        self._running_lock = Lock()

    @property
    def odom(self) -> Optional[Odometry]:
        """Get current odometry."""
        with self._odom_lock:
            return self._odom

    @odom.setter
    def odom(self, value: Odometry) -> None:
        """Set current odometry."""
        with self._odom_lock:
            self._odom = value

    @property
    def pose(self) -> tuple:
        """Get (x, y, z, yaw) tuple. Returns zeros if no odometry."""
        with self._odom_lock:
            if self._odom:
                return (self._odom.x, self._odom.y, self._odom.z, self._odom.yaw)
        return (0.0, 0.0, 0.0, 0.0)

    @property
    def altitude(self) -> float:
        """Get current altitude in meters (positive up)."""
        x, y, z, yaw = self.pose
        return -z  # NED to altitude

    @property
    def selected_target(self) -> Optional[TrackedObject]:
        """Get selected target."""
        with self._target_lock:
            return self._selected_target

    @selected_target.setter
    def selected_target(self, value: TrackedObject) -> None:
        """Set selected target."""
        with self._target_lock:
            self._selected_target = value

    @property
    def running(self) -> bool:
        """Check if mission is running."""
        with self._running_lock:
            return self._running

    def stop(self) -> None:
        """Signal mission to stop."""
        with self._running_lock:
            self._running = False

    def reset(self) -> None:
        """Reset state for new mission."""
        self._running = True
        self.inventory.clear()
        self._selected_target = None
        self.phase_data.clear()
