# Multi-Camera Object Detection Architecture

This document describes the architecture and capabilities enabled by running object detection across multiple cameras simultaneously.

## Overview

A drone equipped with multiple cameras (front, back, down) can leverage YOLO-World detection on each feed to enable capabilities beyond what a single camera provides. This document explores these capabilities and proposes an architecture for multi-camera detection and fusion.

## Camera Configuration

The reference drone (`robot_quadrotor_vision.jsonc`) has three cameras:

| Camera | Position | Orientation | Primary Use |
|--------|----------|-------------|-------------|
| FrontCamera | `0.30, 0, -0.05` | Pitch -15° (tilted down) | Navigation, target acquisition |
| BackCamera | `-0.15, 0, 0` | Yaw 180° (facing rear) | Situational awareness, handoff |
| DownCamera | `0, 0, 0.05` | Pitch -90° (facing down) | Precision landing, overhead view |

```
        Front Camera
        (forward, -15° pitch)
              ▲
              │
              │
    ┌─────────┴─────────┐
    │                   │
    │      DRONE        │
    │                   │
    └─────────┬─────────┘
              │
              ▼
        Back Camera
        (rear, 180° yaw)

              ●
        Down Camera
        (nadir, -90° pitch)
```

## Capabilities

### 1. Efficient 360° Scanning

**Single Camera (Front only):**
- Requires 360° rotation to scan full horizon
- ~12 seconds at 0.5 rad/s scan rate

**Dual Camera (Front + Back):**
- Only 180° rotation needed for full coverage
- ~6 seconds scan time (50% reduction)
- Front and back process simultaneously

```
Single Camera Scan:          Dual Camera Scan:

    ┌───┐                        ┌───┐
    │ F │──360°──▶               │F B│──180°──▶
    └───┘                        └───┘

    Time: 12s                    Time: 6s
```

### 2. Continuous Target Tracking During Maneuvers

**Problem:** When approaching a target, the drone may need to maneuver (avoid obstacle, reposition for landing), causing the target to exit the front camera's field of view.

**Solution:** As target exits front FOV, back camera acquires it. Target is never lost.

```
Time T1: Approaching          Time T2: Flying past         Time T3: Positioning

    ┌───┐                         ┌───┐                        ┌───┐
    │ F │ ──▶ ●                   │   │                        │ B │
    │ B │     Target              │   │ ● ◀── │ B │            │   │ ──▶ ●
    └───┘                         └───┘       └───┘            └───┘

    Front tracking               Handoff                      Back tracking
```

**Use Cases:**
- Fly past target, turn around, land facing away
- Orbit around target while maintaining visual lock
- Complex approach patterns for obstacle avoidance

### 3. Lost Target Recovery

**Problem:** Target lost from front camera due to:
- Wind gust pushing drone off course
- Aggressive obstacle avoidance maneuver
- Tracking failure / occlusion

**Solution:** Other cameras may still see the target. Automatic search across all cameras.

```
┌─────────────────────────────────────────────────────────┐
│  Front Camera: Target LOST                              │
│  Back Camera:  Target detected at bearing +0.3         │
│                                                         │
│  Action: Rotate 180° to reacquire with front camera    │
│          OR continue approach using back camera         │
└─────────────────────────────────────────────────────────┘
```

### 4. Simultaneous Navigation and Rear Safety

**Problem:** During approach, threats or obstacles may appear from behind (other drones, birds, moving vehicles).

**Solution:** Front camera handles navigation while back camera monitors rear hemisphere.

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  Front Camera Role:                                     │
│    - Track target                                       │
│    - Detect obstacles ahead                             │
│    - Guide approach                                     │
│                                                         │
│  Back Camera Role:                                      │
│    - Detect following objects                           │
│    - Monitor cleared path (verify no new obstacles)     │
│    - Alert if threat approaches from rear               │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

**Detection Prompts by Camera:**
```python
front_prompts = ["orange ball", "obstacle", "person"]
back_prompts = ["drone", "bird", "vehicle", "person"]
```

### 5. Down Camera for Terminal Precision

**Problem:** Horizontal cameras estimate target position from bearing angles. Errors accumulate over distance. Final approach requires precision.

**Solution:** When directly above target, down camera provides pixel-perfect positioning with minimal geometric error.

```
Phase: NAVIGATE (Horizontal)     Phase: DESCEND (Vertical)

Front Camera:                    Down Camera:
- Bearing to target              - Target in center = directly above
- Distance estimated             - Offset in pixels = position error
- Some geometric error           - Minimal error when overhead

    ┌───┐                            ┌───┐
    │ F │ ───────▶ ●                 │ D │
    └───┘         Target             └─┬─┘
                                       │
                                       ▼
                                       ●  Target
```

**Precision Landing Workflow:**
1. Front camera guides horizontal approach to within 10m
2. Descend to intermediate altitude (15m)
3. Down camera acquires target
4. Horizontal correction using down camera bearing
5. Vertical descent with continuous down camera tracking
6. Land within 0.5m of target

### 6. Simultaneous Search and Track

**Problem:** While engaging one target, situational awareness of other objects is lost.

**Solution:** Assign cameras different roles - one tracks current target, others continue scanning.

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  Front Camera: TRACK mode                               │
│    - Locked on "orange ball" at bearing (0.1, -0.2)    │
│    - Guiding approach                                   │
│                                                         │
│  Back Camera: SCAN mode                                 │
│    - Detecting: "orange ball" (2), "person" (1)        │
│    - Building inventory for next mission phase         │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 7. Stereo Triangulation for Distance Estimation

**Problem:** Single camera can only estimate distance from apparent object size. Accuracy depends on knowing actual object size.

**Solution:** With two cameras viewing the same object, triangulate actual 3D position using known camera baseline.

```
                    ● Target (unknown distance)
                   /│\
                  / │ \
                 /  │  \
                /   │   \
               /    │    \
              /     │     \
           Front   │    Back
           Camera  │    Camera
              ▼    │    ▼
           bearing │  bearing
              α    │    β
                   │
            ◄──────┴──────►
              Baseline: 0.45m

    Distance = baseline / (tan(α) + tan(β))
```

**Accuracy Improvement:**
- Single camera: ±30% distance error (size estimation)
- Stereo triangulation: ±5% distance error (geometric)

**Limitations:**
- Requires same object visible in both cameras
- Works best when object is to the side (both cameras can see it)
- Not applicable when object is directly ahead or behind

### 8. Verification and Redundancy

**Problem:** Single camera detection may have false positives or miss detections.

**Solution:** Require confirmation from multiple cameras before acting.

```
Detection Confidence Boost:

  Front only:     conf = 0.7     → Action threshold: 0.8 (NO ACTION)
  Front + Back:   conf = 0.7²    → Boosted: 0.85 (ACTION)

Cross-Validation:

  Front detects "orange ball" at world position (50, 20)
  Back detects "orange ball" at world position (48, 21)

  Positions agree within 3m → HIGH CONFIDENCE detection
  Positions disagree by 20m → Possible false positive, investigate
```

## Architecture

### System Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Zenoh Network                                  │
└─────────────────────────────────────────────────────────────────────────┘
        │              │              │                    │
        ▼              ▼              ▼                    ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│ AirSim       │ │ YOLO         │ │ Fusion       │ │ Mission      │
│ Bridge       │ │ Multi-Camera │ │ Node         │ │ Node         │
├──────────────┤ ├──────────────┤ ├──────────────┤ ├──────────────┤
│ Publishes:   │ │ Subscribes:  │ │ Subscribes:  │ │ Subscribes:  │
│ • front/rgb  │─▶ • front/rgb │ │ • det/front  │ │ • targets    │
│ • back/rgb   │─▶ • back/rgb  │ │ • det/back   │ │ • odom       │
│ • down/rgb   │─▶ • down/rgb  │ │ • det/down   │ │              │
│ • odom       │ │              │ │ • odom       │ │ Publishes:   │
│              │ │ Publishes:   │ │              │ │ • cmd/vel    │
│ Subscribes:  │ │ • det/front  │─▶ Publishes:  │ │              │
│ • cmd/vel    │ │ • det/back   │ │ • targets    │─▶              │
└──────────────┘ │ • det/down   │ └──────────────┘ └──────────────┘
                 └──────────────┘
```

### Zenoh Topics

#### Camera Feeds (Published by Bridge)
```
robot/{id}/sensor/camera/front/rgb    # Front camera RGB
robot/{id}/sensor/camera/back/rgb     # Back camera RGB
robot/{id}/sensor/camera/down/rgb     # Down camera RGB
```

#### Per-Camera Detections (Published by Multi-Camera Detector)
```
robot/{id}/perception/detections/front   # Detections from front camera
robot/{id}/perception/detections/back    # Detections from back camera
robot/{id}/perception/detections/down    # Detections from down camera
```

#### Fused Targets (Published by Fusion Node)
```
robot/{id}/perception/targets            # World-frame target list
robot/{id}/perception/tracked/{id}       # Individual tracked object
```

### Data Types

#### CameraDetection (extends DetectionList)

```python
@dataclass
class CameraDetectionList(DetectionList):
    """Detections from a specific camera."""
    camera_id: str              # "front", "back", "down"
    camera_position: Tuple[float, float, float]   # x, y, z relative to drone
    camera_orientation: Tuple[float, float, float] # roll, pitch, yaw
```

#### TrackedTarget (world-frame)

```python
@dataclass
class TrackedTarget:
    """A target tracked across cameras in world frame."""
    target_id: int              # Persistent ID across frames
    class_name: str             # "orange ball"
    confidence: float           # Fused confidence

    # World position (NED)
    world_x: float
    world_y: float
    world_z: float
    position_uncertainty: float  # Estimated error in meters

    # Velocity estimate
    velocity_x: float
    velocity_y: float
    velocity_z: float

    # Visibility
    visible_in: List[str]       # ["front", "down"]
    last_seen_us: int           # Timestamp

    # For control
    bearing_from_front: Optional[float]  # For yaw control
    bearing_from_down: Optional[float]   # For position control
```

#### TargetList

```python
@dataclass
class TargetList:
    """All tracked targets."""
    timestamp_us: int
    targets: List[TrackedTarget]

    def get_by_class(self, class_name: str) -> List[TrackedTarget]
    def get_by_id(self, target_id: int) -> Optional[TrackedTarget]
    def get_visible_in(self, camera: str) -> List[TrackedTarget]
```

### Fusion Node Responsibilities

1. **Coordinate Transform**: Convert per-camera bearings to world positions using drone pose and camera geometry

2. **Association**: Match detections across cameras (same object seen by front and back)

3. **Triangulation**: When object visible in multiple cameras, compute precise 3D position

4. **Tracking**: Maintain persistent IDs across frames using Kalman filter or similar

5. **Confidence Fusion**: Combine detection confidences from multiple cameras

6. **Handoff Management**: Detect when target is leaving one camera's FOV and entering another's

## Implementation Phases

### Phase 1: Multi-Camera Detector
- Extend `yolo_world_detector.py` to process multiple camera feeds
- Publish per-camera detections on separate topics
- Support different prompts per camera

### Phase 2: Basic Fusion
- Create `detection_fusion.py` node
- Transform detections to world frame
- Simple nearest-neighbor association
- Publish fused target list

### Phase 3: Enhanced Mission
- Update mission to subscribe to fused targets
- Implement camera handoff during approach
- Use down camera for terminal guidance

### Phase 4: Advanced Features
- Stereo triangulation for distance
- Kalman filter tracking with persistent IDs
- Multi-target tracking

## Usage Example

```bash
# Terminal 1: AirSim bridge
python airsim_zenoh_bridge.py --connect tcp/localhost:7447

# Terminal 2: Multi-camera YOLO detector
python yolo_multi_camera_detector.py --connect tcp/localhost:7447 \
    --cameras front back down \
    --prompts "orange ball"

# Terminal 3: Detection fusion
python detection_fusion.py --connect tcp/localhost:7447

# Terminal 4: Mission
python fly_to_orange_ball_multicam.py --connect tcp/localhost:7447
```

## Performance Considerations

### GPU Memory
- Each camera stream requires ~500MB GPU memory for yolov8s-world
- 3 cameras = ~1.5GB GPU memory
- Consider sequential processing if GPU-limited

### Latency
| Component | Single Camera | Three Cameras (Parallel) | Three Cameras (Sequential) |
|-----------|---------------|--------------------------|----------------------------|
| Inference | 15ms | 15ms | 45ms |
| Fusion | - | 2ms | 2ms |
| **Total** | 15ms | 17ms | 47ms |

### Bandwidth
- RGB 640x480 @ 30Hz ≈ 27 MB/s per camera
- 3 cameras = 81 MB/s
- Zenoh shared memory eliminates copy overhead on same machine

## References

- [YOLO-World Multi-Task](https://docs.ultralytics.com/models/yolo-world/)
- [Multi-Object Tracking (SORT)](https://arxiv.org/abs/1602.00763)
- [Camera Geometry and Triangulation](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
