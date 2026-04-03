# Isaac Sim Integration Architecture Assessment

**Date**: 2026-04-02
**Status**: Proposal — awaiting experiment results for final decision

## Context

This document proposes three possible architectures for integrating NVIDIA Isaac Sim into the auto-drone platform, enabling virtual development, testing, and optimization of autonomy functionality for drones. The environment must support:

- **Short-range flexible rotary aircraft** (multirotors/quadcopters)
- **Long-range winged aircraft** (fixed-wing drones)
- **High-speed FPV racing drones**

### What Exists Today

A mature Zenoh-based autonomy stack (Phases 1-5 complete) with C++ libs (data types, PID control, behavior trees, perception) and a Python AirSim bridge that connects to Project AirSim on Windows via Zenoh. The `isaac_client/` directory is commented out and unimplemented.

### What Isaac Sim Offers

- **USD scene format** — universal scene description for all assets
- **OmniGraph** — visual dataflow/computation graph framework
- **RTX-accelerated sensors** — camera (RGB/depth/segmentation), lidar, radar
- **PhysX 5** — GPU-accelerated rigid-body physics
- **Headless / WebRTC rendering** — supports remote visualization
- **ZMQ bridge** (`IsaacSimZMQ`) — open-source bridge with Protobuf serialization, cleanly separates transport from message logic
- **Drone physics** — community extensions like Pegasus Simulator (multirotors); fixed-wing and FPV racing require custom aerodynamic models

---

## Architecture A: "Zenoh-Native Extension" (Deep Integration)

**Concept**: Write a custom Isaac Sim extension that embeds Zenoh directly inside the simulator process. OmniGraph nodes publish sensor data to Zenoh topics and subscribe to command topics — no intermediate bridge process.

```text
+---------------------------------------------------+
|  Isaac Sim Process (GPU host or cloud)             |
|                                                    |
|  USD Scene --> OmniGraph Pipeline                  |
|                   |                                |
|          +--------+--------+                       |
|          | Zenoh Extension | (custom OmniGraph     |
|          |   (C++/Python)  |  nodes with CUDA      |
|          +--------+--------+  pointer access)      |
|                   |                                |
|              Zenoh SHM                             |
|      (zero-copy for same-machine)                  |
+-------------------+-------------------------------+
                    | Zenoh pub/sub
                    | (network for remote)
+-------------------+-------------------------------+
|  Autonomy Host (dev laptop / edge device)          |
|                                                    |
|  libs/zenoh_interface --> control_algorithms        |
|  autonomy_stack (SLAM, perception, BT)             |
|  mission_framework (Python phases)                 |
+----------------------------------------------------+
```

### Key Design Decisions

- C++ OmniGraph nodes get CUDA pointers to rendered frames, copy into Zenoh SHM buffer, zero-copy to subscriber on same machine
- Python OmniGraph nodes for prototyping (slower, but simpler)
- Vehicle dynamics as separate extensions: Pegasus for multirotors, custom `FixedWingDynamics` and `FPVRacerDynamics` extensions applying forces per physics step
- Zenoh key expressions map 1:1 to existing conventions (`robot/{id}/sensor/camera/rgb`, etc.)

### Strengths

- Lowest latency possible — no bridge process, no serialization boundary for same-machine
- Zenoh SHM aligns with the documented strategy for real-time SLAM with image streams
- Single communication protocol end-to-end; no translation layers
- Naturally supports existing Zenoh topic conventions and data types

### Weaknesses

- Tight coupling to Isaac Sim's extension API — breaks on major Omniverse API changes (they renamed all extensions in 4.5 to 5.0)
- Requires building Zenoh-C inside the Omniverse extension build system (non-trivial)
- Harder to test without a running Isaac Sim instance
- No reuse of existing IsaacSimZMQ work from NVIDIA

---

## Architecture B: "Zenoh-ZMQ Bridge" (Adapt IsaacSimZMQ)

**Concept**: Fork NVIDIA's open-source `IsaacSimZMQ` and replace the ZMQ transport with Zenoh, keeping the Protobuf message schemas. A separate bridge process translates between Isaac Sim's native format and the Zenoh data types.

```text
+---------------------------------------------------+
|  Isaac Sim Process                                 |
|                                                    |
|  USD Scene --> OmniGraph --> IsaacSimZMQ ext        |
|                                (C++ CUDA path)     |
|                                    |               |
|                               ZMQ socket           |
+------------------------------------+---------------+
                                     | (localhost or TCP)
+------------------------------------+---------------+
|  Bridge Process  (Python or C++)                   |
|                                                    |
|  ZMQ subscriber --> Protobuf decode                |
|                        |                           |
|                  data_types conversion              |
|                        |                           |
|                  Zenoh publisher                    |
|                  (robot/{id}/sensor/*)              |
|                                                    |
|  Zenoh subscriber <-- Protobuf encode              |
|  (robot/{id}/cmd/*) --> ZMQ publisher              |
+----------------------------------------------------+
                    | Zenoh pub/sub
+-------------------+--------------------------------+
|  Autonomy Host                                     |
|  (unchanged from current architecture)             |
+----------------------------------------------------+
```

### Key Design Decisions

- Keep IsaacSimZMQ's C++ OmniGraph nodes (CUDA frame access, Protobuf serialization) untouched
- Bridge process is conceptually identical to the existing `airsim_zenoh_bridge.py` — same pattern, new simulator
- Protobuf `.proto` files from IsaacSimZMQ define the contract; bridge converts to `libs/data_types` serialization
- Can run bridge co-located with Isaac Sim (ZMQ localhost) or with autonomy host (ZMQ networked)

### Strengths

- Fastest path to working integration — IsaacSimZMQ handles sensor acquisition, CUDA-to-CPU transfer, and serialization
- Follows the same pattern as the proven `airsim_zenoh_bridge.py`
- Decoupled: Isaac Sim API changes only affect the bridge, not autonomy code
- Can run Isaac Sim on a remote GPU machine with the bridge co-located there
- Protobuf schemas are well-documented and versioned

### Weaknesses

- Extra serialization hop: CUDA -> Protobuf -> ZMQ -> deserialize -> re-serialize -> Zenoh
- Bridge process adds ~1-3ms latency per message
- Cannot use Zenoh SHM across the ZMQ boundary
- Two communication protocols to maintain and debug
- Vehicle dynamics still need Pegasus/custom extensions inside Isaac Sim

---

## Architecture C: "Headless API + Zenoh" (Standalone Python Control)

**Concept**: Run Isaac Sim headless via its Python API (the Core Experimental API in 5.0+). A Python process controls the simulation loop directly, reads sensor data from render products, computes physics, and publishes everything to Zenoh. No extensions, no OmniGraph, no bridge.

```text
+--------------------------------------------------------+
|  Isaac Sim Headless Process (Python-controlled)         |
|                                                         |
|  +-----------------------------------------------------+
|  |  Python Control Script                               |
|  |                                                      |
|  |  sim = SimulationApp(headless=True)                  |
|  |  world = World()                                     |
|  |  drone = world.scene.add(ArticulationView(...))      |
|  |                                                      |
|  |  while sim.is_running():                             |
|  |      world.step()                                    |
|  |      rgb = camera.get_rgba()                         |
|  |      imu = imu_sensor.get_current_frame()            |
|  |      zenoh_session.put("robot/drone/sensor/...",     |
|  |                        serialize(rgb))               |
|  |      cmd = zenoh_session.get("robot/.../cmd")        |
|  |      drone.apply_action(cmd)                         |
|  |                                                      |
|  |  +----------------------------------------------+    |
|  |  |  Vehicle Dynamics Module                     |    |
|  |  |  - MultirotorModel                          |    |
|  |  |  - FixedWingModel                            |    |
|  |  |  - FPVRacerModel                             |    |
|  |  +----------------------------------------------+    |
|  +-----------------------------------------------------+
+-------------------------------+------------------------+
                                | Zenoh
+-------------------------------+------------------------+
|  Autonomy Host                                          |
|  (unchanged)                                            |
|                                                         |
|  Optional: WebRTC viewer connects to Isaac Sim for      |
|  visualization on a separate machine                    |
+---------------------------------------------------------+
```

### Key Design Decisions

- Isaac Sim's Python API controls the simulation step loop — full timing control
- Vehicle dynamics are pure Python classes (not extensions), applied as forces via `rigid_body.apply_force_and_torque()` each physics step
- Sensor data read via `camera.get_rgba()`, `camera.get_depth()` — returns numpy arrays, directly serializable
- Zenoh-Python (`zenoh` pip package) publishes/subscribes in the same process
- WebRTC streaming enabled separately for remote visualization (purely optional, decoupled from data path)

### Strengths

- Simplest architecture — pure Python, no extension build system, no OmniGraph, no bridge process
- Easiest to experiment with different vehicle dynamics models (just swap a Python class)
- Full control over simulation stepping — can pause, fast-forward, deterministic replay
- Natural fit for RL training loops (Isaac Lab uses this exact pattern)
- Vehicle dynamics models are testable outside Isaac Sim (unit test the math independently)
- WebRTC viewer is fully decoupled from the data path

### Weaknesses

- Python GIL limits throughput — cannot parallelize sensor reads and Zenoh publishes
- `get_rgba()` returns CPU numpy arrays — no CUDA zero-copy path
- ~5-10ms overhead per camera frame vs. C++ OmniGraph path
- Less "production-grade" for high-frequency control loops (>100Hz)
- Zenoh SHM requires the `shared-memory` feature flag (`pip install eclipse-zenoh --no-binary :all: --config-settings build-args="--features=zenoh/shared-memory"`)
- Visualization requires separate WebRTC connection — no integrated viewport debugging

---

## Vehicle Support Strategy

### Architecture A & B (Isaac Sim Extensions)

| Vehicle Type | Dynamics Source | Sensor Suite |
|---|---|---|
| Multirotor (short-range) | Pegasus Simulator extension (rotor-level control) | RTX camera, IMU, GPS, barometer, lidar |
| Fixed-wing (long-range) | Custom extension: lift/drag forces as f(AoA, airspeed) applied at CoP | RTX camera, IMU, GPS, pitot tube (custom) |
| FPV racing drone | Pegasus base + custom high-thrust motor model, reduced drag | Wide-angle/fisheye RTX camera, IMU, minimal lidar |

### Architecture C (Pure Python)

| Vehicle Type | Dynamics Implementation |
|---|---|
| Multirotor | Python class: 4-6 rotor positions, `thrust = k * omega^2`, `torque = b * omega^2`. ~50 lines. |
| Fixed-wing | Python class: `lift = 0.5 * rho * v^2 * S * CL(alpha)`, drag similar. Control surfaces as torque generators. ~100 lines. |
| FPV racing | Multirotor subclass with higher thrust-to-weight ratio, reduced drag coefficients, response time tuning. |

---

## Comparison Matrix

| Criterion | A: Zenoh Extension | B: ZMQ Bridge | C: Headless API |
|---|---|---|---|
| **Latency** | Lowest (~0.5ms SHM) | Medium (~3-5ms) | Higher (~5-10ms) |
| **Implementation effort** | High (extension SDK) | Medium (fork IsaacSimZMQ) | Low (Python scripts) |
| **Maintenance burden** | High (Omniverse API churn) | Medium (bridge is isolated) | Low (stable Python API) |
| **Experimentation speed** | Slow (rebuild extension) | Medium | Fast (edit and rerun) |
| **Vehicle model flexibility** | Medium (C++/Python ext) | Medium | High (pure Python) |
| **Production readiness** | Highest | High | Medium |
| **Remote rendering** | Built-in viewport | WebRTC separate | WebRTC separate |
| **Zenoh SHM support** | Yes (C++ path) | No (ZMQ boundary) | Yes (requires `shared-memory` feature flag) |
| **Similarity to existing code** | New pattern | Mirrors airsim_bridge | New pattern |
| **Multi-sim support** | Isaac only | Could bridge both AirSim and Isaac | Isaac only |

---

## Recommended Experimentation Plan

Start with Architecture C to validate vehicle dynamics and sensor pipelines quickly, then graduate to B or A for production performance.

### Experiment 1: Headless Multirotor (Architecture C) — 1-2 weeks

**Goal**: Validate that Isaac Sim can drive the existing autonomy stack via Zenoh.

1. Create `sim_interfaces/isaac_headless/` with a Python script that:
   - Launches Isaac Sim headless
   - Loads a simple USD scene (flat ground + sky)
   - Spawns a rigid body "drone" with 4 force application points
   - Implements a `MultirotorDynamics` class (thrust/torque from rotor speeds)
   - Publishes camera RGB + IMU to Zenoh (`robot/drone/sensor/camera/rgb`, `robot/drone/sensor/imu`)
   - Subscribes to velocity commands (`robot/drone/cmd/velocity`)
2. Run the existing `demos/02_waypoint_following` WaypointManager against it
3. **Success criteria**: drone follows waypoints in Isaac Sim, frames visible in `viewer_node_headless`

### Experiment 2: Fixed-Wing and FPV Models (Architecture C) — 1 week

**Goal**: Validate that the same Zenoh interface supports radically different vehicle types.

1. Implement `FixedWingDynamics` (lift/drag as f(alpha, v)) and `FPVRacerDynamics` (high-TWR multirotor)
2. Load appropriate USD meshes (wing aircraft, racing quad)
3. Publish to same Zenoh topics but with vehicle-specific `{id}` (`robot/fixed_wing/...`, `robot/fpv_racer/...`)
4. Write a simple fixed-wing controller (pitch to altitude, roll to heading) as a new control algorithm
5. **Success criteria**: all three vehicle types fly with the same autonomy-side code structure

### Experiment 3: ZMQ Bridge Performance (Architecture B) — 1 week

**Goal**: Measure whether the ZMQ bridge overhead is acceptable for real-time requirements.

1. Fork `IsaacSimZMQ`, get it running with the Experiment 1 scene
2. Write an `isaac_zenoh_bridge.py` (modeled on `airsim_zenoh_bridge.py`) that bridges ZMQ to Zenoh
3. Measure end-to-end latency: Isaac Sim render to Zenoh subscriber callback
4. Compare with Experiment 1 direct-publish latency
5. **Success criteria**: < 10ms camera frame latency, < 2ms IMU latency

### Experiment 4: Remote Rendering (Architecture B or C) — 1 week

**Goal**: Validate the split deployment: Isaac Sim on GPU machine, autonomy on dev laptop.

1. Run Isaac Sim in Docker with WebRTC streaming enabled
2. Run autonomy code on a separate machine, connected via Zenoh network
3. Connect WebRTC viewer from a browser
4. Run a full mission (e.g., fly-to-orange-ball) across the network split
5. **Success criteria**: mission completes with acceptable latency, WebRTC provides usable visualization

### Decision Gate

After experiments 1-4, data will inform the final architecture choice:

- If C's latency is acceptable -> stay with C (simplest)
- If < 5ms needed with same-machine deployment -> invest in A (Zenoh extension)
- If remote deployment with good performance needed -> invest in B (ZMQ bridge)
- Most likely outcome: **B for production, C for rapid prototyping and RL training**

---

## Proposed Directory Structure

```text
sim_interfaces/
  isaac_headless/              # Experiment 1-2 (Architecture C)
    isaac_zenoh_node.py        # Main headless sim + Zenoh pub/sub
    vehicle_dynamics/
      __init__.py
      multirotor.py            # 4/6/8-rotor force model
      fixed_wing.py            # Lift/drag aerodynamic model
      fpv_racer.py             # High-performance multirotor variant
    scenes/
      flat_ground.usd          # Basic test environment
      urban_canyon.usd         # Obstacle-rich environment
      race_track.usd           # FPV racing course
    README.md

  isaac_zmq_bridge/            # Experiment 3 (Architecture B)
    isaac_zenoh_bridge.py      # ZMQ <-> Zenoh translation
    proto/                     # Protobuf schemas (from IsaacSimZMQ)
    README.md

  isaac_client/                # Future (Architecture A, if needed)
    ...                        # Zenoh extension for Isaac Sim
```

---

## References

- [Isaac Sim Reference Architecture](https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/reference_architecture.html)
- [IsaacSimZMQ GitHub Repository](https://github.com/isaac-sim/IsaacSimZMQ)
- [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)
- [Isaac Sim Container Deployment](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html)
- [Isaac Sim Sensor Documentation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_camera.html)
