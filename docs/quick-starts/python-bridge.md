# Python bridge to Project AirSim

The current Project AirSim only has a python client, so we need to create a bridge that can translate our Zenoh messages into AirSim messages.

```verbatim
  [Workstation A - Windows]                      [Workstation B - Linux]
  ┌─────────────────────────────────────┐        ┌─────────────────────────────────┐
  │  Project AirSim                     │        │  Autonomy Stack                 │
  │  (Simulator)                        │        │                                 │
  │       ↕ (localhost only)            │        │  - Object Tracking              │
  │  Python AirSim Bridge               │        │  - SLAM                         │
  │       ↕                             │        │  - Path Planning                │
  │  Zenoh Publisher/Subscriber         │        │  - Control Algorithms           │
  │       ↕                             │        │       ↕                         │
  │  [Zenoh Network Transport]  ←───────────────→│  [Zenoh Subscriber/Publisher]   │
  └─────────────────────────────────────┘        └─────────────────────────────────┘
```

  On Workstation A (Windows): Python service that:
  - Connects to Project AirSim via localhost (using projectairsim Python client)
  - Publishes sensor data (RGB, depth, odometry) to Zenoh topics
  - Subscribes to Zenoh command topics and forwards to AirSim

  On Workstation B (Linux): Your existing autonomy code:
  - Subscribes to sensor data via Zenoh
  - Publishes velocity commands via Zenoh

  This keeps the same Zenoh topic conventions you've already defined (robot/drone/sensor/camera/rgb, robot/drone/cmd/velocity, etc.), just with the bridge running in Python on the Windows side instead of C++.