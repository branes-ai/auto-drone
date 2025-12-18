#!/usr/bin/env python3
"""
YOLO-World Multi-Camera Object Detector Node for Zenoh.

Subscribes to multiple camera feeds, runs YOLO-World inference on each,
and publishes per-camera detections.

Supports:
- Multiple cameras (front, back, down) with separate topics
- Different detection prompts per camera
- Parallel or sequential inference (configurable)
- Camera-specific detection publishing

Usage:
    # All cameras with same prompts
    python yolo_multi_camera_detector.py --connect tcp/localhost:7447 \
        --cameras front back down \
        --prompts "orange ball"

    # Different prompts per camera
    python yolo_multi_camera_detector.py --connect tcp/localhost:7447 \
        --cameras front back \
        --front-prompts "orange ball" "obstacle" \
        --back-prompts "person" "vehicle"

Requirements:
    pip install ultralytics>=8.1.0 zenoh opencv-python numpy
"""

import argparse
import sys
import time
import threading
from typing import Optional, List, Dict
from dataclasses import dataclass, field

import numpy as np
import cv2
import zenoh

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Install with: pip install ultralytics>=8.1.0")

from data_types import (
    ImageData, Detection, DetectionList, DetectorConfig, CameraDetectionList
)


@dataclass
class CameraConfig:
    """Configuration for a single camera."""
    camera_id: str                          # "front", "back", "down"
    prompts: List[str] = field(default_factory=lambda: ["orange ball"])
    enabled: bool = True

    # Camera geometry (for world-frame transforms)
    position: tuple = (0.0, 0.0, 0.0)       # x, y, z relative to drone
    orientation: tuple = (0.0, 0.0, 0.0)    # roll, pitch, yaw in degrees


# Default camera configurations matching robot_quadrotor_vision.jsonc
DEFAULT_CAMERA_CONFIGS = {
    "front": CameraConfig(
        camera_id="front",
        position=(0.30, 0.0, -0.05),
        orientation=(0.0, -15.0, 0.0)  # Tilted 15° down
    ),
    "back": CameraConfig(
        camera_id="back",
        position=(-0.15, 0.0, 0.0),
        orientation=(0.0, 0.0, 180.0)  # Facing rear
    ),
    "down": CameraConfig(
        camera_id="down",
        position=(0.0, 0.0, 0.05),
        orientation=(0.0, -90.0, 0.0)  # Facing down
    ),
}


class CameraProcessor:
    """Handles image processing for a single camera."""

    def __init__(self, config: CameraConfig, model: YOLO, detection_config: DetectorConfig):
        self.config = config
        self.model = model
        self.detection_config = detection_config

        # State
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_time: float = 0.0
        self.image_lock = threading.Lock()
        self.frame_count = 0
        self.last_inference_time = 0.0

        # Set prompts for this camera
        self._prompts_set = False

    def set_prompts(self, prompts: List[str]):
        """Update prompts for this camera."""
        self.config.prompts = prompts
        self._prompts_set = False  # Will be set before next inference

    def update_image(self, image: np.ndarray):
        """Update the latest image for this camera."""
        with self.image_lock:
            self.latest_image = image
            self.latest_image_time = time.time()

    def get_image(self) -> Optional[np.ndarray]:
        """Get latest image if not stale."""
        with self.image_lock:
            if self.latest_image is None:
                return None
            # Skip if image is stale (> 500ms)
            if time.time() - self.latest_image_time > 0.5:
                return None
            return self.latest_image.copy()

    def process(self) -> Optional[CameraDetectionList]:
        """Run detection on latest image."""
        image = self.get_image()
        if image is None:
            return None

        # Ensure prompts are set
        if not self._prompts_set:
            self.model.set_classes(self.config.prompts)
            self._prompts_set = True

        height, width = image.shape[:2]
        start_time = time.time()

        # Run inference
        results = self.model.predict(
            image,
            conf=self.detection_config.confidence_threshold,
            iou=self.detection_config.nms_threshold,
            max_det=self.detection_config.max_detections,
            verbose=False
        )

        inference_time = (time.time() - start_time) * 1000
        self.last_inference_time = inference_time

        # Convert results
        detections = []
        result = results[0]

        if result.boxes is not None:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])

                bbox_x, bbox_y = int(x1), int(y1)
                bbox_w, bbox_h = int(x2 - x1), int(y2 - y1)
                center_x = bbox_x + bbox_w // 2
                center_y = bbox_y + bbox_h // 2

                bearing_x = (center_x - width / 2) / (width / 2)
                bearing_y = (center_y - height / 2) / (height / 2)

                class_name = self.config.prompts[cls_id] if cls_id < len(self.config.prompts) else f"class_{cls_id}"

                detections.append(Detection(
                    class_id=cls_id,
                    class_name=class_name,
                    confidence=conf,
                    bbox_x=bbox_x,
                    bbox_y=bbox_y,
                    bbox_w=bbox_w,
                    bbox_h=bbox_h,
                    center_x=center_x,
                    center_y=center_y,
                    area=bbox_w * bbox_h,
                    bearing_x=bearing_x,
                    bearing_y=bearing_y,
                ))

        self.frame_count += 1

        return CameraDetectionList(
            timestamp_us=int(time.time() * 1_000_000),
            frame_id=self.frame_count,
            image_width=width,
            image_height=height,
            inference_time_ms=inference_time,
            camera_id=self.config.camera_id,
            camera_position=self.config.position,
            camera_orientation=self.config.orientation,
            detections=detections
        )


class YoloMultiCameraDetector:
    """
    Multi-camera YOLO-World detector node.

    Processes multiple camera feeds and publishes per-camera detections.
    """

    def __init__(
        self,
        connect_endpoint: str,
        robot_id: str = "drone",
        model_name: str = "yolov8s-world.pt",
        cameras: Optional[List[str]] = None,
        default_prompts: Optional[List[str]] = None,
        confidence: float = 0.25,
        device: str = ""
    ):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.model_name = model_name
        self.device = device
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # Default configuration
        self.detection_config = DetectorConfig(
            prompts=default_prompts or ["orange ball"],
            confidence_threshold=confidence,
        )

        # Camera configurations
        self.cameras = cameras or ["front"]
        self.camera_configs: Dict[str, CameraConfig] = {}
        self.camera_processors: Dict[str, CameraProcessor] = {}

        # Model (shared across cameras for efficiency)
        self.model: Optional[YOLO] = None
        self.model_ready = False

        # Processing mode
        self.sequential_mode = True  # Process cameras sequentially (saves GPU memory)

        # Statistics
        self.total_frames = 0
        self.fps = 0.0

    def load_model(self) -> bool:
        """Load YOLO-World model."""
        if not YOLO_AVAILABLE:
            print("ERROR: ultralytics package not available")
            return False

        try:
            print(f"Loading YOLO-World model: {self.model_name}")
            self.model = YOLO(self.model_name)

            # Set default prompts
            print(f"Default prompts: {self.detection_config.prompts}")
            self.model.set_classes(self.detection_config.prompts)

            # Warm up
            print("Warming up model...")
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False)

            self.model_ready = True
            print(f"Model ready on device: {self.model.device}")
            return True

        except Exception as e:
            print(f"ERROR loading model: {e}")
            return False

    def setup_cameras(self, camera_prompts: Optional[Dict[str, List[str]]] = None):
        """Set up camera configurations and processors."""
        camera_prompts = camera_prompts or {}

        for cam_id in self.cameras:
            # Get default config or create basic one
            if cam_id in DEFAULT_CAMERA_CONFIGS:
                config = DEFAULT_CAMERA_CONFIGS[cam_id]
            else:
                config = CameraConfig(camera_id=cam_id)

            # Override prompts if specified
            if cam_id in camera_prompts:
                config.prompts = camera_prompts[cam_id]
            else:
                config.prompts = self.detection_config.prompts.copy()

            self.camera_configs[cam_id] = config
            self.camera_processors[cam_id] = CameraProcessor(
                config, self.model, self.detection_config
            )

            print(f"  Camera '{cam_id}': prompts={config.prompts}")

    def connect(self) -> bool:
        """Connect to Zenoh and set up subscriptions."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")
            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')
            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            # Subscribe to each camera topic
            for cam_id in self.cameras:
                # Support both new multi-camera topics and legacy single topic
                topics = [
                    f"robot/{self.robot_id}/sensor/camera/{cam_id}/rgb",  # New format
                ]

                # For 'front' camera, also try legacy topic
                if cam_id == "front":
                    topics.append(f"robot/{self.robot_id}/sensor/camera/rgb")

                for topic in topics:
                    self.session.declare_subscriber(
                        topic,
                        lambda sample, cid=cam_id: self._on_image(sample, cid)
                    )
                    print(f"  Subscribed to: {topic}")

            return True

        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_image(self, sample: zenoh.Sample, camera_id: str):
        """Handle incoming image for a camera."""
        try:
            img_data = ImageData.deserialize(bytes(sample.payload))
            image = img_data.to_numpy()

            if camera_id in self.camera_processors:
                self.camera_processors[camera_id].update_image(image)

        except Exception:
            pass

    def process_all_cameras(self) -> Dict[str, CameraDetectionList]:
        """Process all cameras and return detections."""
        results = {}

        for cam_id, processor in self.camera_processors.items():
            # Set prompts before processing (in case they changed)
            self.model.set_classes(processor.config.prompts)

            detections = processor.process()
            if detections:
                results[cam_id] = detections

        return results

    def publish_detections(self, camera_id: str, detections: CameraDetectionList):
        """Publish detections for a camera."""
        topic = f"robot/{self.robot_id}/perception/detections/{camera_id}"
        self.session.put(topic, detections.serialize())

    def run(self):
        """Main processing loop."""
        self.running = True

        print("\n" + "=" * 60)
        print("YOLO-World Multi-Camera Detector")
        print("=" * 60)
        print(f"  Model: {self.model_name}")
        print(f"  Cameras: {self.cameras}")
        print(f"  Confidence threshold: {self.detection_config.confidence_threshold}")
        for cam_id, config in self.camera_configs.items():
            print(f"  {cam_id}: prompts={config.prompts}")
        print("=" * 60 + "\n")

        last_status_time = time.time()
        frames_since_status = 0
        min_interval = 1.0 / self.detection_config.target_fps

        while self.running:
            loop_start = time.time()

            # Process all cameras
            all_detections = self.process_all_cameras()

            # Publish detections
            for cam_id, detections in all_detections.items():
                self.publish_detections(cam_id, detections)
                frames_since_status += 1

            self.total_frames += len(all_detections)

            # Status update every 5 seconds
            now = time.time()
            if now - last_status_time > 5.0:
                self.fps = frames_since_status / (now - last_status_time)
                last_status_time = now
                frames_since_status = 0

                # Build status summary
                status_parts = []
                for cam_id, processor in self.camera_processors.items():
                    det_count = 0
                    if cam_id in all_detections:
                        det_count = len(all_detections[cam_id].detections)
                    status_parts.append(f"{cam_id}:{det_count}")

                print(f"[{self.fps:.1f} FPS] detections: {', '.join(status_parts)}")

            # Rate limiting
            elapsed = time.time() - loop_start
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)

    def stop(self):
        """Stop the detector."""
        self.running = False

    def disconnect(self):
        """Close Zenoh session."""
        if self.session:
            self.session.close()
            self.session = None


def main():
    parser = argparse.ArgumentParser(
        description="YOLO-World multi-camera detector for Zenoh",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single camera (legacy compatible)
  python yolo_multi_camera_detector.py --connect tcp/localhost:7447

  # Multiple cameras with same prompts
  python yolo_multi_camera_detector.py --connect tcp/localhost:7447 \\
      --cameras front back down \\
      --prompts "orange ball"

  # Different prompts per camera
  python yolo_multi_camera_detector.py --connect tcp/localhost:7447 \\
      --cameras front back \\
      --front-prompts "orange ball" "obstacle" \\
      --back-prompts "person" "drone"
        """
    )
    parser.add_argument("--connect", required=True, help="Zenoh endpoint")
    parser.add_argument("--robot-id", default="drone", help="Robot ID")
    parser.add_argument("--model", default="yolov8s-world.pt",
                       help="YOLO-World model")
    parser.add_argument("--cameras", nargs="+", default=["front"],
                       help="Cameras to process (default: front)")
    parser.add_argument("--prompts", nargs="+", default=["orange ball"],
                       help="Default detection prompts")
    parser.add_argument("--confidence", type=float, default=0.25,
                       help="Confidence threshold")
    parser.add_argument("--fps", type=float, default=10.0,
                       help="Target FPS")
    parser.add_argument("--device", default="",
                       help="Device: '' (auto), 'cpu', 'cuda', 'mps'")

    # Per-camera prompts
    parser.add_argument("--front-prompts", nargs="+",
                       help="Prompts for front camera")
    parser.add_argument("--back-prompts", nargs="+",
                       help="Prompts for back camera")
    parser.add_argument("--down-prompts", nargs="+",
                       help="Prompts for down camera")

    args = parser.parse_args()

    if not YOLO_AVAILABLE:
        print("ERROR: ultralytics package required. Install with:")
        print("  pip install ultralytics>=8.1.0")
        return 1

    # Build per-camera prompts
    camera_prompts = {}
    if args.front_prompts:
        camera_prompts["front"] = args.front_prompts
    if args.back_prompts:
        camera_prompts["back"] = args.back_prompts
    if args.down_prompts:
        camera_prompts["down"] = args.down_prompts

    detector = YoloMultiCameraDetector(
        connect_endpoint=args.connect,
        robot_id=args.robot_id,
        model_name=args.model,
        cameras=args.cameras,
        default_prompts=args.prompts,
        confidence=args.confidence,
        device=args.device
    )

    detector.detection_config.target_fps = args.fps

    # Load model
    if not detector.load_model():
        return 1

    # Setup cameras
    detector.setup_cameras(camera_prompts)

    # Connect to Zenoh
    if not detector.connect():
        return 1

    # Run
    try:
        detector.run()
    except KeyboardInterrupt:
        print("\nInterrupted.")

    detector.stop()
    detector.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
