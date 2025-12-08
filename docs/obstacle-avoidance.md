# Obstacle avoidance (Phase 4)

## Demonstration: Obstacle Avoidance

The Obstacle Avoidance demonstration introduces **reactive safety** and **sensor fusion** into the Zenoh architecture. This phase focuses on processing immediate proximity data (e.g., Lidar, depth cameras) and overriding nominal path planning with high-priority safety commands.

### 1. Key Components \& Data Flow

This demonstration requires the integration of a new sensor stream and the introduction of a **Safety Override Node**.

1.  **Simulator Client (New Publisher):** Publishes the proximity/depth sensor data (e.g., Lidar scan, simulated ultrasonic distance readings) to a new Zenoh key.

2.  **Obstacle Detection Node (Subscriber/Publisher):** A new, highly-reactive node. It subscribes to the proximity data, quickly identifies potential collision vectors, and publishes a prioritized, *corrective* velocity command (`robot/drone/cmd/reactive_vel`).

3.  **Command Arbiter (New Node):** This is the core safety component. It subscribes to all primary motion commands (e.g., from Waypoint Following, Object Tracking, etc.) and the high-priority corrective commands from the Obstacle Detection Node. It uses a defined **arbitration logic** (e.g., safety commands always override nominal commands) and publishes the *final* accepted command to the Simulator Client.

4.  **Simulator Client (Final Subscriber):** Subscribes *only* to the final command from the Command Arbiter.

### 2. New Libraries and Data Types

#### A. New Data Types (`libs/data_types/`)

 * **`ProximityData.h`**: Structure for sensor fusion input. For simplicity, we can use an array of distance readings or a structured point cloud snippet.

   ```cpp
   struct ProximityData {
       std::vector<float> distances; // E.g., 360 readings from a Lidar
       // ... time stamp, sensor position, etc.
       // ... serialization methods
   };
   ```

 * **`PrioritizedCommand.h`**: A structure for the Arbiter to understand the priority of incoming velocity commands.

   ```cpp
   struct PrioritizedCommand {
       ControlCommand command; // The velocity command structure from Phase 2
       int priority_level;     // E.g., 1 (Nominal), 10 (Avoidance), 100 (Emergency Stop)
   };
   ```

#### B. Control Algorithms (`libs/control_algorithms/`)

 * **`ReactiveAvoidance.h`**: Logic to convert `ProximityData` into a `ControlCommand` that steers the robot away from the nearest obstacle. This is usually a simple geometric calculation.

 * **`CommandArbiter.h`**: Logic for implementing the switching mechanism (Arbiter). This is a critical piece of reusable infrastructure.

### 3. Zenoh Key Space for Obstacle Avoidance

This phase formalizes the use of separate keys for nominal control and reactive control, which are then fused by the Arbiter.

| Component | Zenoh Key | Data Type | Publisher | Subscriber | Priority |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Proximity Input** | `robot/drone/sensor/proximity` | `ProximityData` | Simulator Client | Obstacle Detection | N/A |
| **Nominal Command** | `robot/drone/cmd/nominal_vel` | `PrioritizedCommand` | Waypoint/Object Tracker | Command Arbiter | Low |
| **Reactive Command** | `robot/drone/cmd/reactive_vel` | `PrioritizedCommand` | Obstacle Detection | Command Arbiter | High |
| **Final Command** | `robot/drone/cmd/velocity` | `ControlCommand` | Command Arbiter | Simulator Client | N/A |

**Crucial Change:** All nodes that generate high-level movement (Waypoint Tracker, Object Tracker) must now publish to the **nominal key**, and they must wrap their command in the `PrioritizedCommand` structure. The Simulator Client must now only subscribe to the **final key**.

### 4. CMake Integration for Phase 4

We introduce two new core executables: `obstacle_node` and `arbiter_node`.

#### `autonomy_stack/obstacle_avoidance/CMakeLists.txt`

```cmake
# --- 1. Obstacle Detection Node Executable ---
add_executable(obstacle_node 
   ObstacleNode.cpp
)

target_link_libraries(obstacle_node PRIVATE
   zenoh_interface
   data_types
   control_algorithms::reactive # Links the avoidance math
)
```

#### `libs/control_algorithms/CMakeLists.txt` (Update)

```cmake
# Create the Command Arbiter library for reuse
add_library(control_algorithms::arbiter STATIC
   CommandArbiter.cpp
   CommandArbiter.h
)

# Update other libraries with new dependencies if needed
```

#### `demos/03_obstacle_avoidance/CMakeLists.txt`

```cmake
# --- 2. Command Arbiter Executable ---
add_executable(arbiter_node 
   ArbiterNode.cpp
)

target_link_libraries(arbiter_node PRIVATE
   zenoh_interface
   data_types
   control_algorithms::arbiter
)
```

### Summary of Initial R&D Infrastructure

By completing these four phases, we have all the core elements required for advanced robotics R&D without ROS:

1.  **Zenoh Core:** Stable, high-performance data fabric for all data types.
2.  **Simulation Interface:** Reliable connection between Linux/Zenoh and AirSim/Isaac.
3.  **Core Libraries:** Data types, serialization, PID controllers, and platform-specific drivers.
4.  **Autonomy Loop:** Demonstrated perception (`Object Tracking`), planning (`Waypoint Following`), and safety (`Obstacle Avoidance`).
5.  **CMake Structure:** Modular build system ready to expand into `visual_slam`, `visual_nav`, and `swarm_coordination`.

With these set of architectural patterns, we now have a solid foundation. The next logical step is to explore how to expand this foundation into the more sophisticated autonomy features, specifically **Visual SLAM** and **Visual Waypoint Navigation**.





