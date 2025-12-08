\# Object Tracking



\## Demonstration: Object Tracking



The Object Tracking demonstration introduces a \*\*Perception Stack\*\* abstraction and integrates it with the existing control loop. This phase is crucial for establishing how real-time, high-bandwidth sensor data (images) is processed and transformed into actionable, low-bandwidth information (object locations).



\### 1. Key Components \& Data Flow



This system introduces the \*\*Perception Node\*\* and modifies the \*\*Platform Controller\*\* to act on perception-derived goals rather than predefined waypoints.



1\.  \*\*Simulator Client (Publisher):\*\* (Unchanged from Phase 1) Publishes raw camera images (`robot/drone/sensor/camera/rgb`).

2\.  \*\*Perception Node (Subscriber/Publisher):\*\* A new standalone executable. It subscribes to the raw images, runs a minimal object detection/tracking algorithm (e.g., a lightweight YOLO or simple color tracking), and publishes the resulting object data, typically a bounding box and/or a 3D target coordinate.

3\.  \*\*Target Tracker (Platform Controller, modified):\*\* Subscribes to the high-level \*\*Object Location\*\* data and the robot's \*\*Odometry\*\*. Instead of tracking a fixed waypoint, it calculates the necessary movement commands (`ControlCommand`) to keep the tracked object centered in the frame or at a fixed relative distance.



\### 2. New Libraries and Data Types



\#### A. New Data Types (`libs/data\_types/`)



&nbsp; \* \*\*`ObjectDetection.hpp`\*\*: Structure defining the output of the perception system.

&nbsp;   ```cpp

&nbsp;   struct ObjectDetection {

&nbsp;       std::string object\_id;

&nbsp;       float confidence;

&nbsp;       float bbox\_x\_min, bbox\_y\_min, bbox\_x\_max, bbox\_y\_max; // Pixel coordinates

&nbsp;       float world\_x, world\_y, world\_z; // Estimated 3D world coordinates (Crucial for control)

&nbsp;       // ... serialization methods

&nbsp;   };

&nbsp;   ```



\#### B. Control Algorithms (`libs/control\_algorithms/`)



&nbsp; \* \*\*`TargetTrackingFollower.hpp`\*\*: A specialization of the control logic. Instead of calculating error relative to a fixed world point, this class calculates the error relative to the desired image center or desired distance to the object's 3D coordinates. This usually involves a proportional controller on the pixel error and a distance-based controller on the depth/range.



\#### C. Perception Logic (`autonomy\_stack/object\_tracking/`)



This is the first module in the high-level `autonomy\_stack/`. It encapsulates the image processing and the geometry required to project 2D bounding boxes into estimated 3D world coordinates.



&nbsp; \* \*\*`PerceptionEngine.hpp`\*\*: Class handling image ingestion, object detection, and 2D-to-3D projection using the drone's known pose and camera intrinsics.



\### 3. Zenoh Key Space for Object Tracking



| Component | Zenoh Key | Data Type | Publisher | Subscriber |

| :--- | :--- | :--- | :--- | :--- |

| \*\*Raw Image Input\*\* | `robot/drone/sensor/camera/rgb` | Raw Image Bytes | Simulator Client | Perception Node |

| \*\*Perception Output\*\* | `robot/drone/perception/objects` | `ObjectDetection` | Perception Node | Target Tracker |

| \*\*Control Input\*\* | `robot/drone/cmd/velocity` | `ControlCommand` | Target Tracker | Simulator Client |



\*\*Key Design Point:\*\* Note the difference in bandwidth. The \*\*Raw Image\*\* key is high-bandwidth (many MB/s), while the \*\*Perception Output\*\* key is very low-bandwidth (a few bytes per detection). Zenoh is ideally suited to handle this mix on the same data fabric.



\### 4. CMake Integration for Phase 3



We introduce the `autonomy\_stack` directory structure and its dependencies.



\#### `autonomy\_stack/object\_tracking/CMakeLists.txt`



```cmake

\# --- 1. Perception Node Executable ---

add\_executable(perception\_node 

&nbsp;   PerceptionNode.cpp

)



target\_link\_libraries(perception\_node PRIVATE

&nbsp;   zenoh\_interface            # Zenoh communication

&nbsp;   data\_types                 # For Odometry and ObjectDetection serialization

&nbsp;   opencv                     # Image processing and basic detection

&nbsp;   autonomy\_stack::perception # The 2D->3D projection logic

)



\# --- 2. Create the Perception Library ---

add\_library(autonomy\_stack::perception STATIC

&nbsp;   PerceptionEngine.cpp

&nbsp;   PerceptionEngine.h

)

\# Link any necessary geometric libraries or future NN inference frameworks here

```



\#### `demonstrations/02\_object\_tracking/CMakeLists.txt`



```cmake

\# --- 3. Target Tracker Executable (Modified Controller) ---

add\_executable(target\_tracker\_node 

&nbsp;   TargetTrackerNode.cpp

)

target\_link\_libraries(target\_tracker\_node PRIVATE

&nbsp;   zenoh\_interface

&nbsp;   data\_types

&nbsp;   control\_algorithms         # Includes TargetTrackingFollower

&nbsp;   drone\_platform

)

```



This structure separates the \*\*sensor stream\*\* (`robot/drone/sensor/...`) from the \*\*derived information\*\* (`robot/drone/perception/...`) and the \*\*command stream\*\* (`robot/drone/cmd/...`), which is essential for scaling to more complex autonomy.



