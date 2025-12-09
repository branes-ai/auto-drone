# Session: Phase 3 - Object Tracking

**Date:** 2025-12-08
**Focus:** Perception pipeline, color-based object detection, visual servoing

## Goals

1. Add ObjectDetection data type for perception results
2. Create autonomy_stack/object_tracking with PerceptionEngine
3. Implement ColorBlobDetector for simple object detection
4. Add TargetTracker control algorithm for visual servoing
5. Build object tracking demo with mock target publisher

## Key Decisions

### Perception Architecture
- Separation of detector (2D) from PerceptionEngine (2D-to-3D)
- Abstract ObjectDetector base class for pluggable detectors
- ColorBlobDetector as simple starting point (red blob detection)
- Future: YOLO, ArUco markers, etc.

### 2D-to-3D Projection
- Camera pinhole model with configurable intrinsics
- Depth estimation from bounding box size (known object width assumption)
- Support for optional depth map input (RGBD cameras)
- NED coordinate frame (X forward, Y right, Z down)

### TargetTracker Control
- Image-based visual servoing (IBVS) approach
- Horizontal pixel error → yaw rate
- Vertical pixel error → vertical velocity
- Distance error → forward velocity
- Configurable PID gains for each axis

### Object Classification
- Class IDs: UNKNOWN, PERSON, VEHICLE, ANIMAL, DRONE, MARKER, COLORED_BLOB
- Designed for future extension with ML-based classifiers

## Implementation

### libs/data_types/
| File | Description |
|------|-------------|
| `ObjectDetection.hpp/cpp` | Detection struct with 2D bbox and 3D position |

### autonomy_stack/object_tracking/
| File | Description |
|------|-------------|
| `PerceptionEngine.hpp/cpp` | 2D-to-3D projection, detector management |
| `PerceptionNode.cpp` | Zenoh subscriber for camera, publisher for detections |
| `CMakeLists.txt` | Library and executable build |

### libs/control_algorithms/
| File | Description |
|------|-------------|
| `TargetTracker.hpp/cpp` | Visual servoing controller |

### demos/03_object_tracking/
| File | Description |
|------|-------------|
| `MockTargetPublisher.cpp` | Generates images with moving red circle |
| `TargetTrackerNode.cpp` | Subscribes to detections, publishes velocity |
| `CMakeLists.txt` | Demo build configuration |

## Code Highlights

### Color Blob Detection (HSV)
```cpp
cv::inRange(hsv,
    cv::Scalar(config_.h_min, config_.s_min, config_.v_min),
    cv::Scalar(config_.h_max, config_.s_max, config_.v_max),
    mask);

// Handle red wraparound (170-180)
if (config_.detect_red_wraparound && config_.h_min < 20) {
    cv::Mat mask2;
    cv::inRange(hsv, cv::Scalar(170, s_min, v_min), cv::Scalar(180, s_max, v_max), mask2);
    cv::bitwise_or(mask, mask2, mask);
}
```

### 2D-to-3D Projection
```cpp
void PerceptionEngine::pixel_to_ray(float px, float py, float& rx, float& ry, float& rz) const {
    float nx = (px - config_.camera.cx) / config_.camera.fx;
    float ny = (py - config_.camera.cy) / config_.camera.fy;
    float len = std::sqrt(nx * nx + ny * ny + 1.0f);
    rx = nx / len;  // Right
    ry = ny / len;  // Down
    rz = 1.0f / len; // Forward
}
```

### Depth from Bounding Box Size
```cpp
float PerceptionEngine::estimate_depth_from_bbox(float bbox_width) const {
    // depth = object_width * fx / bbox_width
    return config_.reference_object_width * config_.camera.fx / bbox_width;
}
```

### Visual Servoing Update
```cpp
// Normalize pixel error to [-1, 1]
float error_x = (bbox_cx - image_cx) / image_cx;
float error_y = (bbox_cy - image_cy) / image_cy;

out_yaw_rate = pid_image_x_.update(dt, 0.0f, -error_x);
out_vz = pid_image_y_.update(dt, 0.0f, -error_y);
out_vx = pid_distance_.update(dt, target_distance, current_depth);
```

## Testing

### Integration Test
```bash
# Terminal 1: Mock target with moving red circle
./build/linux-release/demos/03_object_tracking/mock_target_publisher

# Terminal 2: Perception node (color blob detection)
./build/linux-release/autonomy_stack/object_tracking/perception_node

# Terminal 3: Target tracker (visual servoing)
./build/linux-release/demos/03_object_tracking/target_tracker_node
```

**Results:**
- Mock publisher: 30 FPS images with moving red target
- Perception node: 100% detection rate (238/238 frames)
- Target tracker: Generated 297 velocity commands, smooth tracking
- 3D position estimates: ~2.2m forward, ±0.7m lateral, ±0.7m vertical

## Zenoh Topics

| Topic | Direction | Data Type | Description |
|-------|-----------|-----------|-------------|
| `robot/drone/sensor/camera/rgb` | Subscribe | ImageData | Raw camera images |
| `robot/drone/perception/objects` | Publish | ObjectDetectionList | Detected objects |
| `robot/drone/cmd/velocity` | Publish | VelocityCommand | Tracking commands |

## Data Flow

```
MockTargetPublisher        PerceptionNode           TargetTrackerNode
       │                         │                         │
       │ camera/rgb              │                         │
       ├────────────────────────►│                         │
       │                         │ perception/objects      │
       │                         ├────────────────────────►│
       │                         │                         │
       │ sensor/state/odom       │                         │
       ├─────────────────────────┼────────────────────────►│
       │                         │                         │
       │                         │            cmd/velocity │
       │                         │◄────────────────────────┤
```

## Serialization Formats

### ObjectDetection (56 bytes)
```
[object_id:4][class_id:4][confidence:4]
[bbox_x_min:4][bbox_y_min:4][bbox_x_max:4][bbox_y_max:4]
[world_x:4][world_y:4][world_z:4]
[flags:1][padding:3][timestamp:8]
```

### ObjectDetectionList (16 + n*56 bytes)
```
[count:4][frame_id:4][frame_timestamp:8][detections:n*56]
```

## Issues Encountered

### Windows Build
- **Issue:** `std::clamp` requires `<algorithm>` header
- **Solution:** Added `#include <algorithm>` to SimulatedDrone.cpp

## Next Steps

- Phase 4: Obstacle avoidance with proximity sensors
- Add YOLO-based detector for real object classes
- ArUco/AprilTag marker detection for precise localization
- Multi-object tracking with Hungarian algorithm
