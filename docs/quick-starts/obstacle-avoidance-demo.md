# Obstacle Avoidance demo

## Running the demo

Locally, on a headless workstation:
```bash
  Local test (4 terminals):
  # Terminal 1: Mock proximity publisher
  ./build/linux-release/demos/04_obstacle_avoidance/mock_proximity_publisher --oscillate

  # Terminal 2: Obstacle avoidance node
  ./build/linux-release/demos/04_obstacle_avoidance/obstacle_avoidance_node

  # Terminal 3: Command arbiter
  ./build/linux-release/demos/04_obstacle_avoidance/command_arbiter_node

  # Terminal 4: Simulated drone (optional, for velocity feedback)
  ./build/linux-release/demos/02_waypoint_following/simulated_drone
```

With AirSim (pillar corridor):
```bash
  # On AirSim Windows host: load scene_pillar_corridor.jsonc
  # On Linux: run the pillar corridor mission
  python pillar_corridor_mission.py --connect tcp/<windows-ip>:7447
```

## Files associated with Obstacle Avoidance demo

Data Types:
  - libs/data_types/ProximityData.hpp - 6-direction proximity sensor struct
  - libs/data_types/ProximityData.cpp - Serialization (32 bytes)

Control Algorithms:
  - libs/control_algorithms/ReactiveAvoidance.hpp - Repulsive field controller
  - libs/control_algorithms/ReactiveAvoidance.cpp - CLEAR/AVOIDING/CRITICAL states
  - libs/control_algorithms/tests/ReactiveAvoidanceTest.cpp - 8 test cases

Command Arbiter Library:
  - autonomy_stack/obstacle_avoidance/CommandArbiter.hpp - Priority-based arbiter
  - autonomy_stack/obstacle_avoidance/CommandArbiter.cpp - Staleness filtering
  - autonomy_stack/obstacle_avoidance/CMakeLists.txt

Demo 04 Executables:
  - demos/04_obstacle_avoidance/MockProximityPublisher.cpp - Synthetic obstacle data
  - demos/04_obstacle_avoidance/ObstacleAvoidanceNode.cpp - Reactive avoidance node
  - demos/04_obstacle_avoidance/CommandArbiterNode.cpp - Arbiter node
  - demos/04_obstacle_avoidance/CMakeLists.txt

AirSim Integration:
  - sim_interfaces/airsim_zenoh_bridge/sim_config/scene_pillar_corridor.jsonc - Scene config
  - sim_interfaces/airsim_zenoh_bridge/pillar_corridor_mission.py - Test mission script

## Data Flow

```text
  WaypointManager       → cmd/nominal_vel (LOW) ───┐
                                                   ├──→ CommandArbiter ──→ cmd/velocity
  ObstacleAvoidanceNode → cmd/reactive_vel (HIGH)──┘
```
