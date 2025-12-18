# YOLO-World Object Detection Architecture

This document describes the architecture for integrating YOLO-World into the autonomous drone system for open-vocabulary object detection.

## Overview

YOLO-World enables **zero-shot object detection** using text prompts. Instead of being limited to fixed classes from training, it can detect any object described in natural language (e.g., "orange ball", "red car", "person wearing a helmet").

This makes it ideal for:
- Rapid prototyping without collecting training data
- Missions with varying target objects
- Detecting objects not in standard COCO classes

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Zenoh Network                                  │
└─────────────────────────────────────────────────────────────────────────┘
        │                         │                         │
        ▼                         ▼                         ▼
┌───────────────┐        ┌────────────────┐        ┌────────────────┐
│  AirSim       │        │  YOLO-World    │        │  Mission       │
│  Zenoh Bridge │        │  Detector Node │        │  Node          │
├───────────────┤        ├────────────────┤        ├────────────────┤
│ Publishes:    │        │ Subscribes:    │        │ Subscribes:    │
│ • camera/rgb  │───────▶│ • camera/rgb   │        │ • detections   │
│ • state/odom  │        │                │        │ • state/odom   │
│               │        │ Publishes:     │        │                │
│ Subscribes:   │        │ • detections   │───────▶│ Publishes:     │
│ • cmd/velocity│◀───────│               │        │ • cmd/velocity │
└───────────────┘        └────────────────┘        └────────────────┘
                                │
                                ▼
                         ┌────────────────┐
                         │  YOLO-World    │
                         │  Model         │
                         │  (GPU/CPU)     │
                         └────────────────┘
```

## Zenoh Topic Design

### Input Topics (Consumed by Detector)

| Topic | Type | Description |
|-------|------|-------------|
| `robot/{id}/sensor/camera/rgb` | `ImageData` | RGB camera frames |
| `robot/{id}/detector/config` | `DetectorConfig` | Runtime configuration (prompts, thresholds) |

### Output Topics (Published by Detector)

| Topic | Type | Description |
|-------|------|-------------|
| `robot/{id}/perception/detections` | `DetectionList` | All detections in frame |
| `robot/{id}/perception/detections/annotated` | `ImageData` | Debug image with bounding boxes |

### Configuration Topics

| Topic | Type | Description |
|-------|------|-------------|
| `robot/{id}/detector/prompts` | `PromptList` | Text prompts for detection |
| `robot/{id}/detector/status` | `DetectorStatus` | Health, FPS, model info |

## Data Types

### DetectorConfig

```python
@dataclass
class DetectorConfig:
    """Configuration for YOLO-World detector."""
    prompts: List[str]              # Text prompts: ["orange ball", "person"]
    confidence_threshold: float     # Min confidence (0.0-1.0), default 0.25
    nms_threshold: float            # Non-max suppression IoU, default 0.45
    max_detections: int             # Max detections per frame, default 20
    publish_annotated: bool         # Publish debug images, default False
    target_fps: float               # Target inference rate, default 10.0
```

### Detection

```python
@dataclass
class Detection:
    """Single object detection."""
    class_id: int           # Index into prompt list
    class_name: str         # The prompt that matched (e.g., "orange ball")
    confidence: float       # Detection confidence 0.0-1.0

    # Bounding box (pixels)
    bbox_x: int             # Top-left X
    bbox_y: int             # Top-left Y
    bbox_w: int             # Width
    bbox_h: int             # Height

    # Derived values (computed by detector)
    center_x: int           # Bbox center X
    center_y: int           # Bbox center Y
    area: int               # Bbox area in pixels

    # Normalized bearing for control (-1 to +1)
    bearing_x: float        # Horizontal: -1=left, 0=center, +1=right
    bearing_y: float        # Vertical: -1=top, 0=center, +1=bottom
```

### DetectionList

```python
@dataclass
class DetectionList:
    """All detections from a single frame."""
    timestamp_us: int               # Microseconds since epoch
    frame_id: int                   # Sequential frame number
    image_width: int                # Source image width
    image_height: int               # Source image height
    inference_time_ms: float        # Model inference time
    detections: List[Detection]     # All detections

    def get_by_class(self, class_name: str) -> List[Detection]:
        """Filter detections by class name."""
        return [d for d in self.detections if d.class_name == class_name]

    def get_best(self, class_name: str) -> Optional[Detection]:
        """Get highest confidence detection of given class."""
        matches = self.get_by_class(class_name)
        return max(matches, key=lambda d: d.confidence) if matches else None
```

## YOLO-World Detector Node

### Dependencies

```bash
pip install ultralytics>=8.1.0  # Includes YOLO-World support
pip install zenoh
pip install opencv-python
pip install numpy
```

### Model Selection

| Model | Size | Speed (T4 GPU) | Accuracy | Use Case |
|-------|------|----------------|----------|----------|
| `yolov8s-world` | 28MB | ~15ms | Good | Real-time on GPU |
| `yolov8m-world` | 52MB | ~25ms | Better | Balanced |
| `yolov8l-world` | 90MB | ~40ms | Best | Accuracy critical |
| `yolov8x-world` | 140MB | ~60ms | Highest | Offline/batch |

For drone missions, recommend `yolov8s-world` or `yolov8m-world`.

### Node Implementation Outline

```python
class YoloWorldDetector:
    """YOLO-World detector node for Zenoh."""

    def __init__(self, connect_endpoint: str, robot_id: str = "drone"):
        self.robot_id = robot_id
        self.session: Optional[zenoh.Session] = None

        # YOLO-World model
        self.model = YOLO("yolov8s-world.pt")
        self.prompts: List[str] = ["orange ball"]  # Default

        # Topics
        self.topic_rgb = f"robot/{robot_id}/sensor/camera/rgb"
        self.topic_detections = f"robot/{robot_id}/perception/detections"
        self.topic_config = f"robot/{robot_id}/detector/config"

        # State
        self.frame_count = 0
        self.config = DetectorConfig(prompts=self.prompts)

    def set_prompts(self, prompts: List[str]):
        """Update detection prompts (triggers model re-configuration)."""
        self.prompts = prompts
        self.model.set_classes(prompts)

    def _on_image(self, sample: zenoh.Sample):
        """Process incoming camera frame."""
        img_data = ImageData.deserialize(bytes(sample.payload))
        image = img_data.to_numpy()  # BGR numpy array

        # Run YOLO-World inference
        results = self.model.predict(
            image,
            conf=self.config.confidence_threshold,
            iou=self.config.nms_threshold,
            max_det=self.config.max_detections,
            verbose=False
        )

        # Convert to DetectionList
        detections = self._results_to_detections(results[0], image.shape)

        # Publish
        self.session.put(self.topic_detections, detections.serialize())

    def _results_to_detections(self, result, img_shape) -> DetectionList:
        """Convert YOLO results to DetectionList."""
        height, width = img_shape[:2]
        detections = []

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            bbox_x, bbox_y = int(x1), int(y1)
            bbox_w, bbox_h = int(x2 - x1), int(y2 - y1)
            center_x = bbox_x + bbox_w // 2
            center_y = bbox_y + bbox_h // 2

            detections.append(Detection(
                class_id=cls_id,
                class_name=self.prompts[cls_id],
                confidence=conf,
                bbox_x=bbox_x,
                bbox_y=bbox_y,
                bbox_w=bbox_w,
                bbox_h=bbox_h,
                center_x=center_x,
                center_y=center_y,
                area=bbox_w * bbox_h,
                bearing_x=(center_x - width/2) / (width/2),
                bearing_y=(center_y - height/2) / (height/2),
            ))

        return DetectionList(
            timestamp_us=int(time.time() * 1_000_000),
            frame_id=self.frame_count,
            image_width=width,
            image_height=height,
            inference_time_ms=result.speed['inference'],
            detections=detections
        )
```

## Mission Integration

### Updated Mission Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Orange Ball Mission (Updated)                     │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  PHASE 1: ASCEND                                                     │
│    └─ Ascend to observation altitude (25m)                          │
│                                                                      │
│  PHASE 2: CONFIGURE DETECTOR                          [NEW]          │
│    └─ Publish config: prompts=["orange ball", "orange sphere"]      │
│    └─ Wait for detector ready                                        │
│                                                                      │
│  PHASE 3: SCAN                                                       │
│    └─ Rotate 360°                                                    │
│    └─ Subscribe to detections (not raw images)        [CHANGED]     │
│    └─ Collect detected objects with world position estimates        │
│                                                                      │
│  PHASE 4: INVENTORY                                                  │
│    └─ Select best "orange ball" detection by confidence             │
│    └─ No more HSV/circularity heuristics              [SIMPLIFIED]  │
│                                                                      │
│  PHASE 5: NAVIGATE                                                   │
│    └─ Fly over target at cruise altitude                            │
│    └─ Descend and approach                                          │
│    └─ Use live detections for terminal guidance       [NEW]         │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Simplifications

**Before (HSV-based):**
```python
# Complex color thresholding
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
# Morphological operations
# Contour finding
# Circularity calculation
# Orange score calculation
# Multi-factor scoring: circ⁴ × orange × √area
```

**After (YOLO-World):**
```python
# Simple: subscribe to detections, filter by class
detections = self.get_latest_detections()
orange_balls = detections.get_by_class("orange ball")
if orange_balls:
    target = max(orange_balls, key=lambda d: d.confidence)
```

### Terminal Guidance Enhancement

During final approach, use live detections to track target:

```python
def phase5_navigate_with_tracking(self, target: DetectedObject):
    """Navigate with live detection feedback."""

    while not arrived:
        # Get live detection of orange ball
        detections = self.get_latest_detections()
        orange_ball = detections.get_best("orange ball")

        if orange_ball and dist_xy < 20.0:
            # Use visual feedback for fine approach
            # bearing_x/y come directly from YOLO detection
            yaw_rate = 0.5 * orange_ball.bearing_x
            # Adjust velocity based on detection position
        else:
            # Fall back to waypoint navigation
            ...
```

## File Structure

```
sim_interfaces/airsim_zenoh_bridge/
├── data_types.py                    # Add Detection, DetectionList
├── yolo_world_detector.py           # YOLO-World detector node  [NEW]
├── fly_to_orange_ball_yolo.py       # Updated mission           [NEW]
└── configs/
    └── detector_config.yaml         # Default prompts, thresholds
```

## Usage

### Start the System

```bash
# Terminal 1: AirSim + Zenoh Bridge
python airsim_zenoh_bridge.py --connect tcp/localhost:7447

# Terminal 2: YOLO-World Detector
python yolo_world_detector.py --connect tcp/localhost:7447 \
    --prompts "orange ball" "orange sphere" \
    --model yolov8s-world.pt \
    --confidence 0.3

# Terminal 3: Mission
python fly_to_orange_ball_yolo.py --connect tcp/localhost:7447
```

### Runtime Prompt Changes

Prompts can be changed at runtime via Zenoh:

```python
# From any node: change what to detect
config = DetectorConfig(prompts=["red car", "stop sign"])
session.put("robot/drone/detector/config", config.serialize())
```

## Performance Considerations

### Latency Budget

| Component | Target | Notes |
|-----------|--------|-------|
| Image capture | <33ms | 30 FPS camera |
| Zenoh transfer | <5ms | Local or SHM |
| YOLO inference | <30ms | yolov8s-world on GPU |
| Detection publish | <2ms | Serialization |
| **Total** | **<70ms** | ~14 Hz detection rate |

### GPU vs CPU

| Platform | Model | Inference Time | Recommended |
|----------|-------|----------------|-------------|
| RTX 3080 | yolov8s-world | ~12ms | Yes |
| RTX 2070 | yolov8s-world | ~18ms | Yes |
| Apple M1 | yolov8s-world | ~35ms | Yes (MPS) |
| CPU (i7) | yolov8s-world | ~150ms | Development only |

### Optimization Tips

1. **Reduce resolution**: Resize to 640x480 before inference if camera is higher res
2. **Skip frames**: Process every 2nd or 3rd frame if CPU-bound
3. **Batch prompts**: Set all prompts once, don't change frequently
4. **Use FP16**: Enable half-precision on supported GPUs

## Testing

### Unit Test: Detection Accuracy

```python
def test_orange_ball_detection():
    detector = YoloWorldDetector(prompts=["orange ball"])

    # Load test image with known orange ball
    image = cv2.imread("test_data/orange_ball_scene.png")
    detections = detector.detect(image)

    assert len(detections.get_by_class("orange ball")) >= 1
    ball = detections.get_best("orange ball")
    assert ball.confidence > 0.5
    # Check bounding box is reasonable
    assert 100 < ball.center_x < 540  # Not at edges
```

### Integration Test: End-to-End

```python
def test_mission_with_yolo():
    # Start detector node
    # Start mission
    # Verify phases complete
    # Check final position near target
```

## Future Enhancements

1. **Multi-camera support**: Run detection on front + down cameras
2. **Object tracking**: Add SORT/DeepSORT for consistent IDs across frames
3. **3D position estimation**: Use depth camera + detection for accurate world position
4. **Fine-tuning**: Collect data, fine-tune for specific objects in your scenes
5. **Edge deployment**: Optimize for Jetson Orin / similar edge hardware

## References

- [YOLO-World Paper](https://arxiv.org/abs/2401.17270)
- [Ultralytics YOLO-World Docs](https://docs.ultralytics.com/models/yolo-world/)
- [Zenoh Documentation](https://zenoh.io/docs/)
