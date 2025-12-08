# Visual SLAM

This is the natural progression into sophisticated autonomy. **Visual SLAM (Simultaneous Localization and Mapping)** will replace the simple, potentially inaccurate odometry from the simulator with a highly accurate, map-relative pose estimate, and also create a map of the environment.

The integration strategy for Visual SLAM must account for its intensive resource requirements and its need for low-latency, synchronized sensor data (images and IMU data). Zenoh is ideally suited here, especially leveraging its potential for **Shared Memory (SHM)** transport for high-bandwidth local data.

## Visual SLAM and State Estimation

### 1. New Component: The SLAM Node

The new core component will be the **SLAM Node**, integrated into the `autonomy_stack/visual_slam/` directory. We will base the requirements on a state-of-the-art solution like ORB-SLAM3, which requires image and Inertial Measurement Unit (IMU) data.

| Component | Zenoh Key | Data Type | Publisher | Subscriber | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **IMU Data** | `robot/drone/sensor/imu` | Custom IMU Struct | Simulator Client | SLAM Node | Provides inertial measurements for Visual-Inertial Odometry (VIO). |
| **Camera Feed** | `robot/drone/sensor/camera/rgb` | Raw Image Bytes | Simulator Client | SLAM Node | Provides the visual input for feature tracking and mapping. |
| **SLAM Pose Output** | `robot/drone/state/pose_slam` | `Pose` Struct (Position + Orientation) | SLAM Node | Command Arbiter, Visual Nav Node | High-accuracy, global-frame localization. |
| **Map Data** | `robot/drone/map/landmarks` | Map Data Struct | SLAM Node | **Zenoh Storage** | Stores the generated map (point cloud, keyframes) for persistence/query. |

### 2. Implementation & Data Flow Strategy

#### A. Data Synchronization and Quality

SLAM algorithms require tightly synchronized and time-stamped sensor data.

1.  **Simulator Client Update:** The client must now publish not only the raw image but also the high-rate IMU data (`robot/drone/sensor/imu`). Both streams must be accurately time-stamped by the simulator.
2.  **SLAM Node Subscription:** The SLAM Node will subscribe to both `camera/rgb` and `sensor/imu`. It must implement an internal **Time Synchronizer** (similar to ROS message filters, but built in C++ using Zenoh callbacks) to pair image frames with corresponding IMU packets before feeding them into the SLAM algorithm.

#### B. Leveraging Zenoh for Performance

The raw image stream is the bottleneck.

  * **Implicit Shared Memory (SHM):** Since the Simulator Client and the SLAM Node will likely run on the same powerful Linux machine (the virtualized GPU instance), Zenoh's ability to automatically use shared memory for large payloads is paramount. This drastically reduces the latency and CPU load associated with copying/serializing large image data across the network stack. **This is a key requirement for real-time SLAM.**
  * **Zenoh Storage:** Zenoh's storage primitive will be used to handle the persistent map data. When the SLAM Node finishes a session, it will use `Zenoh::put` to store the final map structure under the key `robot/drone/map/session_X`. Later, a new SLAM session can use `Zenoh::get` (Query) to retrieve this map for re-localization.

#### C. Localization Refinement (Replacing Odometry)

The new state estimate (`robot/drone/state/pose_slam`) will be fed back into the control system:

  * **Command Arbiter Update:** The Arbiter (and its consumers like the Nominal Controller) must be updated to use the highly accurate `pose_slam` key instead of the basic, often drift-prone `odom` key.

### 3. CMake Integration for SLAM

Integrating a complex third-party library like ORB-SLAM3 is the biggest CMake challenge. The strategy is to treat it as an external dependency and link it as a library.

#### A. Third-Party Setup (`third_party/ORB_SLAM3/`)

The recommended approach is to use CMake's **`ExternalProject_Add`** or **`FetchContent`** in a dedicated **`third_party/CMakeLists.txt`** file to download and build ORB-SLAM3 and its dependencies (Pangolin, DBoW2, g2o) if possible, or assume they are pre-installed on the host Linux machine.

Assuming the SLAM library is built and installed locally, we create an imported library target for ease of use:

```cmake
# third_party/CMakeLists.txt (or a dedicated SLAM finder file)
# Find the necessary SLAM components (assuming they are installed/built)
find_package(ORBSLAM3 REQUIRED) 
find_package(Pangolin REQUIRED)
# ... find g2o, DBoW2, etc.

# Define an IMPORTED target for easy linking
add_library(ORBSLAM3::Core INTERFACE IMPORTED)
set_target_properties(ORBSLAM3::Core PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ORBSLAM3_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${ORBSLAM3_INCLUDE_DIRS}"
)
```

#### B. SLAM Node Executable (`autonomy_stack/visual_slam/CMakeLists.txt`)

```cmake
# Create the Visual SLAM Node executable
add_executable(slam_node
    SLAMNode.cpp
)

# Link the executable to the necessary libraries
target_link_libraries(slam_node PRIVATE
    zenoh_interface            # Zenoh communication
    data_types                 # For IMU, Pose serialization
    ORBSLAM3::Core             # The entire SLAM library
)

# Add dependencies for the Time Synchronizer utility headers
target_include_directories(slam_node PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
```

This establishes the final piece of the perception-localization chain. With a robust SLAM output, we are now fully equipped to implement **Visual Waypoint Navigation**, where the robot navigates not just by GPS coordinates, but by features relative to the generated map.

