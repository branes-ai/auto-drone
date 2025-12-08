# Object Tracking

## Demonstration: Object Tracking (Phase 3)

The Object Tracking demonstration introduces a **Perception Stack** abstraction and integrates it with the existing control loop. This phase is crucial for establishing how real-time, high-bandwidth sensor data (images) is processed and transformed into actionable, low-bandwidth information (object locations).

### 1. Key Components & Data Flow

This system introduces the **Perception Node** and modifies the **Platform Controller** to act on perception-derived goals rather than predefined waypoints.



1.  **Simulator Client (Publisher):** (Unchanged from Phase 1) Publishes raw camera images (`robot/drone/sensor/camera/rgb`).

2.  **Perception Node (Subscriber/Publisher):** A new standalone executable. It subscribes to the raw images, runs a minimal object detection/tracking algorithm (e.g., a lightweight YOLO or simple color tracking), and publishes the resulting object data, typically a bounding box and/or a 3D target coordinate.

3.  **Target Tracker (Platform Controller, modified):** Subscribes to the high-level **Object Location** data and the robot's **Odometry**. Instead of tracking a fixed waypoint, it calculates the necessary movement commands (`ControlCommand`) to keep the tracked object centered in the frame or at a fixed relative distance.

### 2. New Libraries and Data Types

#### A. New Data Types (`libs/data_types/`)

 * **`ObjectDetection.h`**: Structure defining the output of the perception system.

   ```cpp
   struct ObjectDetection {
       std::string object_id;
       float confidence;
       float bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max; // Pixel coordinates
       float world_x, world_y, world_z; // Estimated 3D world coordinates (Crucial for control)

       // ... serialization methods
   };
   ```

#### B. Control Algorithms (`libs/control_algorithms/`)

 * **`TargetTrackingFollower.hpp`**: A specialization of the control logic. Instead of calculating error relative to a fixed world point, this class calculates the error relative to the desired image center or desired distance to the object's 3D coordinates. This usually involves a proportional controller on the pixel error and a distance-based controller on the depth/range.

#### C. Perception Logic (`autonomy_stack/object_tracking/`)

This is the first module in the high-level `autonomy_stack/`. It encapsulates the image processing and the geometry required to project 2D bounding boxes into estimated 3D world coordinates.

 * **`PerceptionEngine.hpp`**: Class handling image ingestion, object detection, and 2D-to-3D projection using the drone's known pose and camera intrinsics.

### 3. Zenoh Key Space for Object Tracking

| Component | Zenoh Key | Data Type | Publisher | Subscriber |
| :--- | :--- | :--- | :--- | :--- |
| **Raw Image Input** | `robot/drone/sensor/camera/rgb` | Raw Image Bytes | Simulator Client | Perception Node |
| **Perception Output** | `robot/drone/perception/objects` | `ObjectDetection` | Perception Node | Target Tracker |
| **Control Input** | `robot/drone/cmd/velocity` | `ControlCommand` | Target Tracker | Simulator Client |

**Key Design Point:** Note the difference in bandwidth. The **Raw Image** key is high-bandwidth (many MB/s), while the **Perception Output** key is very low-bandwidth (a few bytes per detection). Zenoh is ideally suited to handle this mix on the same data fabric.

### 4. CMake Integration for Phase 3

We introduce the `autonomy_stack` directory structure and its dependencies.

#### `autonomy_stack/object_tracking/CMakeLists.txt`

```cmake
# --- 1. Perception Node Executable ---
add_executable(perception_node 
   PerceptionNode.cpp
)

target_link_libraries(perception_node PRIVATE
   zenoh_interface            # Zenoh communication
   data_types                 # For Odometry and ObjectDetection serialization
   opencv                     # Image processing and basic detection
   autonomy_stack::perception # The 2D->3D projection logic
)

# --- 2. Create the Perception Library ---
add_library(autonomy_stack::perception STATIC
   PerceptionEngine.cpp
   PerceptionEngine.h
)

# Link any necessary geometric libraries or future NN inference frameworks here
```

#### `demonstrations/02_object_tracking/CMakeLists.txt`

```cmake
# --- 3. Target Tracker Executable (Modified Controller) ---
add_executable(target_tracker_node 
   TargetTrackerNode.cpp
)

target_link_libraries(target_tracker_node PRIVATE
   zenoh_interface
   data_types
   control_algorithms         # Includes TargetTrackingFollower
   drone_platform
)
```

This structure separates the **sensor stream** (`robot/drone/sensor/...`) from the **derived information** (`robot/drone/perception/...`) and the **command stream** (`robot/drone/cmd/...`), which is essential for scaling to more complex autonomy.