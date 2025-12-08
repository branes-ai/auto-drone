\# Waypoint following



\## Demonstration: Way-Point Following



The Way-Point Following demonstration introduces the first \*\*closed-loop control system\*\* in our Zenoh data fabric. This requires defining the command key space and integrating the basic control algorithms.



\### 1. Key Components \& Data Flow



This phase involves three primary nodes communicating via Zenoh:



1\.  \*\*Waypoint Planner (Publisher):\*\* A standalone executable that calculates a series of target points (waypoints) and publishes the \*current desired waypoint\*.

2\.  \*\*Platform Controller (Subscriber/Publisher):\*\* Runs on the robot/simulator side. It subscribes to the desired waypoint, subscribes to the robot's current state (Odometry), calculates the necessary velocity or thrust commands to reach the waypoint, and publishes these commands.

3\.  \*\*Simulator Client (Subscriber):\*\* The same client from Phase 1, but now it \*\*subscribes\*\* to the control commands and translates them into API calls for the simulator (e.g., setting drone velocity in AirSim).



\### 2. New Libraries and Data Types



The essential control components within the `libs/` and `platforms/` directories require clean data types and control command abstractions.



\#### A. New Data Types (`libs/data\_types/`)



&nbsp; \* \*\*`Waypoint.hpp`\*\*: A structure defining a target position, potentially with arrival tolerance and speed constraints.

&nbsp;   ```cpp

&nbsp;   struct Waypoint {

&nbsp;       float x, y, z;

&nbsp;       float yaw\_target; // Desired orientation at waypoint

&nbsp;       // ... serialization methods

&nbsp;   };

&nbsp;   ```

&nbsp; \* \*\*`ControlCommand.hpp`\*\*: A structure for the low-level inputs the simulator expects (e.g., linear velocity and angular velocity).

&nbsp;   ```cpp

&nbsp;   struct ControlCommand {

&nbsp;       float linear\_x, linear\_y, linear\_z;

&nbsp;       float angular\_yaw;

&nbsp;       // ... serialization methods

&nbsp;   };

&nbsp;   ```



\#### B. Control Algorithms (`libs/control\_algorithms/`)



This library will house the generic control logic, such as a basic PID controller used for error reduction.



&nbsp; \* \*\*`PIDController.hpp`\*\*: A generic class to implement Proportional-Integral-Derivative control logic.

&nbsp; \* \*\*`WaypointFollower.hpp`\*\*: A higher-level logic class that uses the PID controller to track a target point. It calculates the error between the current position (from Odometry) and the target (Waypoint) and outputs the desired `ControlCommand`.



\#### C. Platform Interface (`platforms/drone\_platform/`)



This directory now contains the specific logic that adapts the generic `ControlCommand` to the drone's actual state machine and the simulator's specific requirements (e.g., AirSim might require separate calls for setting linear velocity and angular rate).



&nbsp; \* \*\*`DroneController.hpp`\*\*: A class that wraps the `WaypointFollower` and manages the state of the drone (e.g., Armed, Taking Off, Flying, Landing).



\### 3. Zenoh Key Space for Waypoint Following



| Component | Zenoh Key | Data Type | Publisher | Subscriber |

| :--- | :--- | :--- | :--- | :--- |

| \*\*Robot State\*\* | `robot/drone/sensor/state/odom` | `Odometry` | Simulator Client | Platform Controller |

| \*\*Waypoint Target\*\* | `autonomy/waypoint\_topic` | `Waypoint` | Waypoint Planner | Platform Controller |

| \*\*Control Input\*\* | `robot/drone/cmd/velocity` | `ControlCommand` | Platform Controller | Simulator Client |



\### 4. CMake Integration



The CMake setup for the three new executables needs to link the new libraries.



\#### `demonstrations/01\_waypoint\_following/CMakeLists.txt`



This folder now holds two new executables: `planner\_node` and `controller\_node`.



```cmake

\# --- 1. Waypoint Planner Executable ---

add\_executable(planner\_node 

&nbsp;   PlannerNode.cpp

)

target\_link\_libraries(planner\_node PRIVATE

&nbsp;   zenoh\_interface

&nbsp;   data\_types

)



\# --- 2. Platform Controller Executable ---

add\_executable(controller\_node 

&nbsp;   ControllerNode.cpp

)

target\_link\_libraries(controller\_node PRIVATE

&nbsp;   zenoh\_interface

&nbsp;   data\_types

&nbsp;   control\_algorithms

&nbsp;   drone\_platform       # Links the high-level control logic

)



\# Note: The Simulator Client from Phase 1 (now subscribing to velocity) 

\# will also be updated and recompiled with the new dependency on ControlCommand.

```



\### 5. Progression of Logic



1\.  \*\*Initialization:\*\* All three nodes establish their Zenoh sessions and connect.

2\.  \*\*State Reporting:\*\* The \*\*Simulator Client\*\* continuously publishes the current `Odometry` state.

3\.  \*\*Planning:\*\* The \*\*Waypoint Planner\*\* publishes the first `Waypoint` goal.

4\.  \*\*Control Loop:\*\* The \*\*Platform Controller\*\* receives the `Odometry` and the `Waypoint`. It calculates the error, runs its `WaypointFollower` logic, and publishes the resulting `ControlCommand`.

5\.  \*\*Actuation:\*\* The \*\*Simulator Client\*\* receives the `ControlCommand` and sends the appropriate velocity/thrust commands to the simulation environment, causing the drone to move.



This framework allows us to swap out the \*\*Platform Controller\*\* logic in the future (e.g., replace PID with a Reinforcement Learning policy) without changing the \*\*Waypoint Planner\*\* or the \*\*Simulator Client\*\*.



