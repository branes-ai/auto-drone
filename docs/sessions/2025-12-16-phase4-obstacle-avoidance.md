# Session: Phase 4 - Obstacle Avoidance Implementation

**Date:** 2025-12-16
**Focus:** Implementing obstacle avoidance with command arbitration for the autonomy stack

## Goals

1. Implement ProximityData type for 6-direction proximity sensing
2. Create ReactiveAvoidance controller using repulsive potential fields
3. Build CommandArbiter for priority-based velocity command selection
4. Create Demo 04 executables for testing obstacle avoidance
5. Define AirSim scene and mission for pillar corridor testing

## Accomplishments

### 1. ProximityData Type

Created 6-direction proximity sensor data structure:

**Files:**
- `libs/data_types/ProximityData.hpp`
- `libs/data_types/ProximityData.cpp`

**Structure:**
```cpp
struct ProximityData {
    float distance_front = 999.0f;  // meters (999 = no obstacle)
    float distance_back = 999.0f;
    float distance_left = 999.0f;
    float distance_right = 999.0f;
    float distance_up = 999.0f;
    float distance_down = 999.0f;
    uint64_t timestamp_us = 0;

    static constexpr size_t SERIALIZED_SIZE = 32;

    float min_distance() const;
    float min_horizontal_distance() const;
    bool has_obstacle(float threshold = 2.0f) const;
    static ProximityData clear(uint64_t ts = 0);
};
```

**Zenoh Topic:** `robot/drone/sensor/proximity`

### 2. ReactiveAvoidance Controller

Implemented repulsive potential field algorithm:

**Files:**
- `libs/control_algorithms/ReactiveAvoidance.hpp`
- `libs/control_algorithms/ReactiveAvoidance.cpp`
- `libs/control_algorithms/tests/ReactiveAvoidanceTest.cpp`

**States:**
- `CLEAR` - No obstacles within safety distance
- `AVOIDING` - Generating escape velocity
- `CRITICAL` - Obstacle too close, emergency stop

**Algorithm:**
- For each direction with obstacle < safety_distance:
  - Compute repulsive force: `gain * (1/distance - 1/safety_distance)`
  - Direction: away from obstacle
- Sum all repulsive velocities
- Clamp magnitude to max_avoidance_speed

**Configuration:**
```cpp
struct Config {
    float safety_distance = 2.0f;      // Start avoiding
    float critical_distance = 0.5f;    // Emergency stop
    float max_avoidance_speed = 2.0f;  // Max escape velocity
    float gain = 1.5f;                 // Repulsive force gain
};
```

### 3. CommandArbiter Library

Priority-based command selection with staleness filtering:

**Files:**
- `autonomy_stack/obstacle_avoidance/CommandArbiter.hpp`
- `autonomy_stack/obstacle_avoidance/CommandArbiter.cpp`
- `autonomy_stack/obstacle_avoidance/CMakeLists.txt`

**Selection Logic:**
1. Filter out commands older than `command_timeout_us` (default 500ms)
2. Select command with highest `priority` (CRITICAL > HIGH > NORMAL > LOW)
3. On priority tie, prefer most recent timestamp
4. No valid commands → return hover

### 4. Demo 04 Executables

**Files:**
- `demos/04_obstacle_avoidance/CMakeLists.txt`
- `demos/04_obstacle_avoidance/MockProximityPublisher.cpp`
- `demos/04_obstacle_avoidance/ObstacleAvoidanceNode.cpp`
- `demos/04_obstacle_avoidance/CommandArbiterNode.cpp`

**Data Flow:**
```
WaypointManager ──→ cmd/nominal_vel (LOW) ───┐
                                              ├──→ CommandArbiter ──→ cmd/velocity
ObstacleAvoidanceNode → cmd/reactive_vel (HIGH)┘
```

**MockProximityPublisher options:**
- `--direction front|back|left|right|up|down`
- `--initial <dist>` - Starting distance
- `--min-dist <dist>` - Closest approach
- `--speed <val>` - Approach speed
- `--oscillate` - Move back and forth

### 5. Integration Changes

Modified WaypointManager to publish to `cmd/nominal_vel` instead of `cmd/velocity`:
- `demos/02_waypoint_following/WaypointManager.cpp` line 120

Added `COMMAND_ARBITER = 5` to VelocityCommand Source enum.

### 6. AirSim Scene & Mission

**Scene:** `sim_interfaces/airsim_zenoh_bridge/sim_config/scene_pillar_corridor.jsonc`

Corridor layout (30m x 6m):
```
START                                         GOAL
  ↓                                             ↓
[wall]     P1    P2          P3    P4        [wall]
  X=0            X=8         X=16   X=24      X=30

Pillar positions (1m diameter):
  P1: (8, -1.5)  - left
  P2: (8, 1.5)   - right (in path)
  P3: (16, -1.5) - left (in path)
  P4: (24, 1.5)  - right
```

**Mission:** `sim_interfaces/airsim_zenoh_bridge/pillar_corridor_mission.py`
- Publishes waypoints straight through corridor
- Simulates proximity data based on pillar positions
- Monitors progress and reports success/failure

## Test Results

All 45 tests pass:
```
100% tests passed, 0 tests failed out of 45

ProximityData tests: 10 new tests
- default construction
- parameterized construction
- serialization roundtrip
- clear factory
- min_distance
- min_horizontal_distance
- has_obstacle
- has_horizontal_obstacle
- deserialization fails on insufficient data
- serialized size is correct

ReactiveAvoidance tests: 8 new tests
- basic functionality
- CLEAR state
- CRITICAL state
- AVOIDING state (6 directions)
- multi-direction avoidance
- velocity clamping
- closer obstacle stronger response
- reset
```

## Files Created

| File | Purpose |
|------|---------|
| `libs/data_types/ProximityData.hpp` | 6-direction proximity data |
| `libs/data_types/ProximityData.cpp` | Serialization (32 bytes) |
| `libs/control_algorithms/ReactiveAvoidance.hpp` | Repulsive field controller |
| `libs/control_algorithms/ReactiveAvoidance.cpp` | Controller implementation |
| `libs/control_algorithms/tests/ReactiveAvoidanceTest.cpp` | 8 test cases |
| `autonomy_stack/obstacle_avoidance/CMakeLists.txt` | Library build |
| `autonomy_stack/obstacle_avoidance/CommandArbiter.hpp` | Priority arbiter |
| `autonomy_stack/obstacle_avoidance/CommandArbiter.cpp` | Arbiter implementation |
| `demos/04_obstacle_avoidance/CMakeLists.txt` | Demo build |
| `demos/04_obstacle_avoidance/MockProximityPublisher.cpp` | Test data generator |
| `demos/04_obstacle_avoidance/ObstacleAvoidanceNode.cpp` | Avoidance node |
| `demos/04_obstacle_avoidance/CommandArbiterNode.cpp` | Arbiter node |
| `sim_interfaces/.../scene_pillar_corridor.jsonc` | AirSim scene |
| `sim_interfaces/.../pillar_corridor_mission.py` | Test mission |

## Files Modified

| File | Change |
|------|--------|
| `libs/data_types/CMakeLists.txt` | Added ProximityData |
| `libs/data_types/tests/DataTypesTest.cpp` | Added ProximityData tests |
| `libs/data_types/VelocityCommand.hpp` | Added COMMAND_ARBITER enum |
| `libs/control_algorithms/CMakeLists.txt` | Added ReactiveAvoidance, linked data_types |
| `libs/control_algorithms/tests/CMakeLists.txt` | Added ReactiveAvoidanceTest.cpp |
| `CMakeLists.txt` | Enabled obstacle_avoidance and demo 04 |
| `demos/02_waypoint_following/WaypointManager.cpp` | Changed to cmd/nominal_vel |

## Running Demo 04

**Local test (4 terminals):**
```bash
# Terminal 1: Mock proximity
./build/linux-release/demos/04_obstacle_avoidance/mock_proximity_publisher --oscillate

# Terminal 2: Avoidance node
./build/linux-release/demos/04_obstacle_avoidance/obstacle_avoidance_node

# Terminal 3: Command arbiter
./build/linux-release/demos/04_obstacle_avoidance/command_arbiter_node

# Terminal 4: Simulated drone
./build/linux-release/demos/02_waypoint_following/simulated_drone
```

**With AirSim:**
```bash
# Load scene_pillar_corridor.jsonc in AirSim
# Run mission:
python pillar_corridor_mission.py --connect tcp/<windows-ip>:7447
```

## Next Steps

1. Test obstacle avoidance with actual AirSim depth sensors
2. Tune ReactiveAvoidance gains for optimal response
3. Add velocity blending option to CommandArbiter (alternative to pure priority)
4. Begin Phase 5: Visual SLAM with ORB-SLAM3 integration
