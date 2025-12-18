#!/usr/bin/env python3
"""
YOLO-World Object Detector Node for Zenoh.

Subscribes to camera images, runs YOLO-World inference with text prompts,
and publishes detections over Zenoh.

YOLO-World enables zero-shot object detection using natural language prompts.
No training required - just specify what you want to detect.

Usage:
    python yolo_world_detector.py --connect tcp/localhost:7447 \
        --prompts "orange ball" "orange sphere" \
        --model yolov8s-world.pt \
        --confidence 0.3

Requirements:
    pip install ultralytics>=8.1.0 zenoh opencv-python numpy
"""

import argparse
import sys
import time
import threading
from typing import Optional, List

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
    ImageData, Detection, DetectionList, DetectorConfig
)


class YoloWorldDetector:
    """
    YOLO-World detector node for Zenoh.

    Subscribes to RGB camera images, runs YOLO-World inference,
    and publishes detections.
    """

    def __init__(
        self,
        connect_endpoint: str,
        robot_id: str = "drone",
        model_name: str = "yolov8s-world.pt",
        prompts: Optional[List[str]] = None,
        confidence: float = 0.25,
        device: str = ""  # "" = auto, "cpu", "cuda", "mps"
    ):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.model_name = model_name
        self.device = device
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # Configuration
        self.config = DetectorConfig(
            prompts=prompts or ["orange ball"],
            confidence_threshold=confidence,
        )

        # Model (lazy loaded)
        self.model: Optional[YOLO] = None
        self.model_ready = False

        # Topics
        self.topic_rgb = f"robot/{robot_id}/sensor/camera/rgb"
        self.topic_detections = f"robot/{robot_id}/perception/detections"
        self.topic_config = f"robot/{robot_id}/detector/config"
        self.topic_status = f"robot/{robot_id}/detector/status"
        self.topic_annotated = f"robot/{robot_id}/perception/detections/annotated"

        # State
        self.frame_count = 0
        self.last_inference_time = 0.0
        self.fps = 0.0
        self.image_lock = threading.Lock()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_time = 0.0

        # Rate limiting
        self.min_interval = 1.0 / self.config.target_fps
        self.last_process_time = 0.0

    def load_model(self) -> bool:
        """Load YOLO-World model."""
        if not YOLO_AVAILABLE:
            print("ERROR: ultralytics package not available")
            return False

        try:
            print(f"Loading YOLO-World model: {self.model_name}")
            self.model = YOLO(self.model_name)

            # Set detection classes from prompts
            print(f"Setting prompts: {self.config.prompts}")
            self.model.set_classes(self.config.prompts)

            # Warm up with dummy inference
            print("Warming up model...")
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False)

            self.model_ready = True
            print(f"Model ready on device: {self.model.device}")
            return True

        except Exception as e:
            print(f"ERROR loading model: {e}")
            return False

    def set_prompts(self, prompts: List[str]):
        """Update detection prompts."""
        if not prompts:
            return

        self.config.prompts = prompts
        if self.model_ready:
            print(f"Updating prompts: {prompts}")
            self.model.set_classes(prompts)

    def connect(self) -> bool:
        """Connect to Zenoh."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")
            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')
            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            # Subscribe to RGB camera
            self.session.declare_subscriber(self.topic_rgb, self._on_rgb)
            print(f"Subscribed to: {self.topic_rgb}")

            # Subscribe to config updates
            self.session.declare_subscriber(self.topic_config, self._on_config)
            print(f"Subscribed to: {self.topic_config}")

            return True

        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_rgb(self, sample: zenoh.Sample):
        """Handle incoming RGB image."""
        try:
            img_data = ImageData.deserialize(bytes(sample.payload))
            image = img_data.to_numpy()

            with self.image_lock:
                self.latest_image = image
                self.latest_image_time = time.time()

        except Exception as e:
            pass  # Silently ignore parse errors

    def _on_config(self, sample: zenoh.Sample):
        """Handle config updates."""
        try:
            new_config = DetectorConfig.deserialize(bytes(sample.payload))
            print(f"Received config update: {new_config.prompts}")

            self.config = new_config
            self.min_interval = 1.0 / self.config.target_fps

            if self.model_ready:
                self.model.set_classes(self.config.prompts)

        except Exception as e:
            print(f"Config parse error: {e}")

    def process_frame(self, image: np.ndarray) -> DetectionList:
        """Run YOLO-World inference on image."""
        height, width = image.shape[:2]
        start_time = time.time()

        # Run inference
        results = self.model.predict(
            image,
            conf=self.config.confidence_threshold,
            iou=self.config.nms_threshold,
            max_det=self.config.max_detections,
            verbose=False,
            device=self.device if self.device else None
        )

        inference_time = (time.time() - start_time) * 1000  # ms
        self.last_inference_time = inference_time

        # Convert results to detections
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

                # Compute bearing (-1 to +1)
                bearing_x = (center_x - width / 2) / (width / 2)
                bearing_y = (center_y - height / 2) / (height / 2)

                detections.append(Detection(
                    class_id=cls_id,
                    class_name=self.config.prompts[cls_id] if cls_id < len(self.config.prompts) else f"class_{cls_id}",
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

        return DetectionList(
            timestamp_us=int(time.time() * 1_000_000),
            frame_id=self.frame_count,
            image_width=width,
            image_height=height,
            inference_time_ms=inference_time,
            detections=detections
        )

    def draw_detections(self, image: np.ndarray, detections: DetectionList) -> np.ndarray:
        """Draw bounding boxes and labels on image."""
        annotated = image.copy()

        for det in detections.detections:
            # Draw bounding box
            color = (0, 255, 0)  # Green
            cv2.rectangle(
                annotated,
                (det.bbox_x, det.bbox_y),
                (det.bbox_x + det.bbox_w, det.bbox_y + det.bbox_h),
                color, 2
            )

            # Draw label
            label = f"{det.class_name}: {det.confidence:.2f}"
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (det.bbox_x, det.bbox_y - h - 4),
                         (det.bbox_x + w, det.bbox_y), color, -1)
            cv2.putText(annotated, label, (det.bbox_x, det.bbox_y - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            # Draw center crosshair
            cv2.drawMarker(annotated, (det.center_x, det.center_y),
                          color, cv2.MARKER_CROSS, 10, 2)

        return annotated

    def run(self):
        """Main processing loop."""
        self.running = True

        print("\n" + "=" * 60)
        print("YOLO-World Detector Node")
        print("=" * 60)
        print(f"  Model: {self.model_name}")
        print(f"  Prompts: {self.config.prompts}")
        print(f"  Confidence threshold: {self.config.confidence_threshold}")
        print(f"  Target FPS: {self.config.target_fps}")
        print(f"  Publishing to: {self.topic_detections}")
        print("=" * 60 + "\n")

        last_status_time = time.time()
        frames_since_status = 0

        while self.running:
            # Rate limiting
            now = time.time()
            if now - self.last_process_time < self.min_interval:
                time.sleep(0.001)
                continue

            # Get latest image
            with self.image_lock:
                if self.latest_image is None:
                    time.sleep(0.01)
                    continue
                image = self.latest_image.copy()
                image_time = self.latest_image_time

            # Skip if image is stale (> 500ms old)
            if now - image_time > 0.5:
                time.sleep(0.01)
                continue

            self.last_process_time = now

            # Run detection
            detections = self.process_frame(image)

            # Publish detections
            self.session.put(self.topic_detections, detections.serialize())

            # Publish annotated image if configured
            if self.config.publish_annotated:
                annotated = self.draw_detections(image, detections)
                img_data = ImageData.from_numpy(annotated)
                self.session.put(self.topic_annotated, img_data.serialize())

            frames_since_status += 1

            # Status update every 5 seconds
            if now - last_status_time > 5.0:
                self.fps = frames_since_status / (now - last_status_time)
                last_status_time = now
                frames_since_status = 0

                det_count = len(detections.detections)
                det_summary = ", ".join(
                    f"{d.class_name}:{d.confidence:.2f}" for d in detections.detections[:3]
                )
                if len(detections.detections) > 3:
                    det_summary += f" (+{len(detections.detections)-3} more)"

                print(f"[{self.fps:.1f} FPS] {self.last_inference_time:.1f}ms | "
                      f"{det_count} detections: {det_summary or 'none'}")

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
        description="YOLO-World object detector for Zenoh",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with default "orange ball" prompt
  python yolo_world_detector.py --connect tcp/localhost:7447

  # Multiple prompts
  python yolo_world_detector.py --connect tcp/localhost:7447 \\
      --prompts "orange ball" "person" "car"

  # Custom model and confidence
  python yolo_world_detector.py --connect tcp/localhost:7447 \\
      --model yolov8m-world.pt --confidence 0.4

  # Force CPU inference
  python yolo_world_detector.py --connect tcp/localhost:7447 --device cpu
        """
    )
    parser.add_argument("--connect", required=True, help="Zenoh endpoint")
    parser.add_argument("--robot-id", default="drone", help="Robot ID (default: drone)")
    parser.add_argument("--model", default="yolov8s-world.pt",
                       help="YOLO-World model (default: yolov8s-world.pt)")
    parser.add_argument("--prompts", nargs="+", default=["orange ball"],
                       help="Detection prompts (default: 'orange ball')")
    parser.add_argument("--confidence", type=float, default=0.25,
                       help="Confidence threshold (default: 0.25)")
    parser.add_argument("--fps", type=float, default=10.0,
                       help="Target FPS (default: 10.0)")
    parser.add_argument("--device", default="",
                       help="Device: '' (auto), 'cpu', 'cuda', 'mps'")
    parser.add_argument("--annotated", action="store_true",
                       help="Publish annotated images for debugging")

    args = parser.parse_args()

    if not YOLO_AVAILABLE:
        print("ERROR: ultralytics package required. Install with:")
        print("  pip install ultralytics>=8.1.0")
        return 1

    detector = YoloWorldDetector(
        connect_endpoint=args.connect,
        robot_id=args.robot_id,
        model_name=args.model,
        prompts=args.prompts,
        confidence=args.confidence,
        device=args.device
    )

    detector.config.target_fps = args.fps
    detector.config.publish_annotated = args.annotated

    # Load model
    if not detector.load_model():
        return 1

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
