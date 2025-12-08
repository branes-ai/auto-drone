# Visual Waypoint Navigation (Phase 6)

## Visual Waypoint Navigation

With the high-accuracy pose established by the SLAM Node, the system can move beyond simple geometric waypoint following to **Visual Waypoint Navigation (V-Nav)**. V-Nav uses the generated map and landmarks to guide the robot, making it resilient to GPS denial and ideal for indoor or complex environments.

### 1. New Component: Visual Navigation Planner

The main new component is the **Visual Navigation Planner**, replacing the simple Waypoint Planner from Phase 2. This planner operates on the SLAM map data and the high-accuracy SLAM Pose.

| Component | Zenoh Key | Data Type | Publisher | Subscriber | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **SLAM Pose** | `robot/drone/state/pose_slam` | `Pose` Struct | SLAM Node | V-Nav Planner | Provides the robot's high-accuracy global position. |
| **Map Data** | `robot/drone/map/session_X` | Map Data Struct | **Zenoh Storage** | V-Nav Planner | Provides the persistent environment representation for planning. |
| **Path Plan** | `autonomy/path_plan` | `Path` Struct (Sequence of Poses) | V-Nav Planner | **Local Controller** | The optimized route calculated by the pathfinding algorithm. |
| **Final Command** | `robot/drone/cmd/nominal_vel` | `PrioritizedCommand` | Local Controller | Command Arbiter | Low-level velocity commands to execute the path. |

### 2. Implementation & Data Flow Strategy

#### A. The V-Nav Planner (`autonomy_stack/visual_nav/`)

This node performs the high-level pathfinding.

1.  **Map Retrieval:** The planner first queries Zenoh Storage to retrieve the latest map data (`Zenoh::get("robot/drone/map/session_X")`).
2.  **Path Planning:** Given a start pose (from `pose_slam`) and an end goal, the planner uses an algorithm (e.g., A\*, RRT\*) to find an optimal, collision-free path through the map.
3.  **Path Publication:** The full path (a sequence of intermediate target poses) is published as a `Path` structure.

#### B. The Local Controller (Modification)

The old Platform Controller (Waypoint Follower) is renamed and refined into the **Local Controller**.

1.  **Subscription:** Subscribes to the new `autonomy/path_plan` key.
2.  **Tracking Logic:** It takes the global path and breaks it down into small, trackable waypoints. It uses the `pose_slam` input to determine its current location relative to the path and generates `ControlCommand` to track the path curvature, publishing the result to the **Command Arbiter** via the `nominal_vel` key.

#### C. **The Zenoh Path:** Path Planning vs. Reactive Control

This phase confirms the decoupled architecture:

  * **Global Plan:** The V-Nav Planner generates the path over several seconds or minutes.
  * **Local Control:** The Local Controller tracks this path in the 10-100Hz range.
  * **Safety Override:** The Obstacle Avoidance Node (Phase 4) still runs at the highest frequency, ready to publish a high-priority command to the **Command Arbiter** if the Local Controller leads the robot too close to an unmapped or dynamic obstacle.

### 3. New Data Type (`libs/data_types/`)

  * **`Path.h`**: A collection of high-level poses that define the planned route.
    ```cpp
    struct Path {
        std::vector<Pose> waypoints; // Using the Pose struct from SLAM output
        // ...
    };
    ```

### 4. CMake Integration for V-Nav

The V-Nav node requires linking to the SLAM libraries for map interaction and also needs a Path Planning library (e.g., a custom implementation or a geometric library).

#### `autonomy_stack/visual_nav/CMakeLists.txt`

```cmake
# Create the Visual Navigation Planner executable
add_executable(vnav_planner_node
    VNavPlannerNode.cpp
)

target_link_libraries(vnav_planner_node PRIVATE
    zenoh_interface            # Zenoh communication (including Zenoh Storage primitives)
    data_types                 # For Path, Pose serialization
    autonomy_stack::slam       # Library to interface with SLAM map data structures
    path_planning_algos        # Custom or third-party pathfinding library (e.g., OMPL if complex)
)
```