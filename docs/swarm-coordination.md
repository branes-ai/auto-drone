# SWARM Coordination (Phase 7)

## Swarm Coordination and Learning

This final phase leverages Zenoh's decentralized design to enable multi-robot systems and machine learning integration.

### 1. Swarm Coordination

**Zenoh's Strength:** Zenoh is natively multi-peer. You can run multiple instances of the entire pipeline (Simulator Client, SLAM Node, Arbiter, etc.) for *Drone 1*, *Drone 2*, etc., on different machines, all connected to the same Zenoh network.

#### A. Key Space Extension

The key structure inherently supports swarms by using a wildcard or a robot ID:

  * **Individual Keys:** `robot/drone1/state/pose_slam`, `robot/drone2/state/pose_slam`
  * **Swarm Coordination Key:** `swarm/coordinator/task_assignment` (Published by the Coordinator)
  * **Discovery/Query:** A coordinator node can use a Zenoh Query expression like `robot/*/state/pose_slam` to retrieve the poses of *all* robots simultaneously.

#### B. The Swarm Coordinator (`autonomy_stack/swarm_coordinator/`)

This new node subscribes to the pose of all robots and determines the global task, publishing task-specific waypoints (or roles) to each robot's dedicated key:

  * **Input:** Query results for `robot/*/state/pose_slam`.
  * **Output:** Publishes individual waypoints to `robot/drone1/cmd/nominal_vel`, `robot/drone2/cmd/nominal_vel`, etc.

### 2. Machine Learning Integration

For training or real-time inference, the flexibility of the Zenoh data fabric is key.

#### A. Inference Node

  * **ML Inference Node:** A Python/PyTorch node (Zenoh has Python bindings) subscribes to a sensor stream (e.g., `robot/drone/sensor/camera/rgb`). It runs a trained model and publishes the result (e.g., a specific high-level behavioral command) to a new key: `robot/drone/cmd/ml_policy_vel`.
  * The **Command Arbiter** can be updated to handle this new command stream, potentially giving it an even higher priority than the V-Nav output, allowing the ML model to dictate complex behavior.

#### B. Data Collection (Training)

  * A dedicated **Data Logger** node subscribes to *all* streams (`robot/**`) and uses Zenoh's persistent storage or simple file output (JSON/Protobuf/etc.) to save synchronized state/image/command pairs for later use in training a new Embodied AI policy.

This final architecture provides a comprehensive, high-performance R\&D environment ready for both current autonomous tasks and future machine learning integration.
