# Behavior Trees

Successfully added BehaviorTree.CPP integration with Zenoh-only communication (no ZeroMQ/Groot2).

What was built:

C++ Library (libs/behavior_tree/):
  - PhaseAction node - calls Python phases via Zenoh RPC
  - BlackboardSync - syncs Zenoh state with BT blackboard
  - 6 condition nodes: HasTarget, HasDetections, CheckAltitude, CheckDistance, CheckMinAltitude, IsMissionRunning
  - 2 XML tree files: orange_ball_mission.xml, robust_mission.xml

Python Bridge (packages/mission_framework/bt_bridge/):
  - PhaseServer - executes phases on RPC requests
  - StatePublisher - publishes MissionState for C++ blackboard
  - run_server.py - entry point

Demo (demos/05_behavior_tree/):
  - bt_mission_runner - loads and executes behavior trees

Build & Test Results:

  100% tests passed, 0 tests failed out of 53

Usage:

```bash
  # Terminal 1: Start Python phase server
  python -m mission_framework.bt_bridge.run_server --connect tcp/localhost:7447

  # Terminal 2: Run behavior tree mission
  ./build/linux-release/demos/05_behavior_tree/bt_mission_runner \
      --tree trees/orange_ball_mission.xml \
      --connect tcp/localhost:7447
```

The behavior tree layer adds fallbacks, retries, and conditional logic while reusing all existing Python phases unchanged.
