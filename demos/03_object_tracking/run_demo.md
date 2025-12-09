#  Phase 3: Object Tracking - Complete

## New Components

  libs/data_types/
  - ObjectDetection.hpp/cpp - Detection results with 2D bounding box and 3D world position

  autonomy_stack/object_tracking/
  - PerceptionEngine - Camera intrinsics, 2D-to-3D projection, depth estimation
  - ColorBlobDetector - HSV color-based blob detection (red by default)
  - perception_node - Processes camera images, publishes ObjectDetectionList

  libs/control_algorithms/
  - TargetTracker - Visual servoing controller (image error → velocity commands)

  demos/03_object_tracking/
  - mock_target_publisher - Generates images with moving red circular target
  - target_tracker_node - Tracks detected objects, outputs velocity commands

## Zenoh Topics

  | Topic                          | Data Type           | Description                        |
  |--------------------------------|---------------------|------------------------------------|
  | robot/drone/perception/objects | ObjectDetectionList | Detected objects with 3D positions |

## Run the Demo

  # Terminal 1: Mock target publisher
  ./build/linux-release/demos/03_object_tracking/mock_target_publisher

  # Terminal 2: Perception node
  ./build/linux-release/autonomy_stack/object_tracking/perception_node

  # Terminal 3: Target tracker
  ./build/linux-release/demos/03_object_tracking/target_tracker_node

The pipeline successfully:
  1. Detects red blobs in camera images
  2. Estimates 3D position using camera intrinsics and bbox size
  3. Generates velocity commands to track the target while maintaining distance

