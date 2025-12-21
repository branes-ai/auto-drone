# Session: Behavior Tree Integration

**Date:** 2025-12-21
**Focus:** Adding BehaviorTree.CPP for complex mission orchestration

## Goals

1. Add state-of-the-art behavior tree support for complex mission logic
2. Integrate with existing mission framework (phases as leaf nodes)
3. Keep messaging infrastructure clean (Zenoh only, no ZeroMQ)
4. Enable fallbacks, retries, and conditional logic in missions

## Accomplishments

### 1. BehaviorTree.CPP Integration

Added BehaviorTree.CPP v4.6 as a dependency via CMake FetchContent, with all non-essential features disabled to keep dependencies minimal:

```cmake
set(BTCPP_GROOT_INTERFACE OFF)  # No ZeroMQ - we use Zenoh
set(BTCPP_SQLITE_LOGGING OFF)
set(BTCPP_ENABLE_MINITRACE OFF)
```

### 2. C++ Behavior Tree Library

Created `libs/behavior_tree/` with custom nodes for drone missions:

**Condition Nodes:**
| Node | Purpose |
|------|---------|
| `HasTarget` | Check if target is selected |
| `HasDetections` | Check if objects detected (with min_count) |
| `CheckAltitude` | Check if at target altitude (with tolerance) |
| `CheckDistance` | Check if within distance threshold |
| `CheckMinAltitude` | Check if above minimum altitude |
| `IsMissionRunning` | Check mission active flag |

**Action Nodes:**
| Node | Purpose |
|------|---------|
| `PhaseAction` | Execute Python phase via Zenoh RPC |

**Support Classes:**
| Class | Purpose |
|-------|---------|
| `BlackboardSync` | Sync Zenoh state with BT blackboard |
| `FactoryConfig` | Configure factory with Zenoh session |

### 3. Python Bridge

Created `packages/mission_framework/bt_bridge/` for C++ ↔ Python communication:

**Components:**
- `PhaseServer` - Listens for phase requests, executes phases, returns results
- `StatePublisher` - Publishes MissionState to Zenoh for C++ blackboard
- `run_server.py` - Entry point for running the bridge

**Zenoh Topics:**
```
robot/{id}/bt/phase/request/{phase_name}   # C++ → Python: {"params": {...}}
robot/{id}/bt/phase/response/{phase_name}  # Python → C++: {"status": "...", "data": {...}}
robot/{id}/bt/state/target                 # Target info for blackboard
robot/{id}/bt/state/inventory              # Inventory summary for blackboard
robot/{id}/bt/state/mission                # Mission running state
```

### 4. XML Behavior Trees

Created example trees in `libs/behavior_tree/trees/`:

**orange_ball_mission.xml** - Simple sequential mission:
```xml
<Sequence>
    <PhaseAction phase_name="ascend"/>
    <PhaseAction phase_name="scan" params='{"rotation_degrees": 180}'/>
    <PhaseAction phase_name="select"/>
    <HasTarget/>
    <PhaseAction phase_name="navigate"/>
    <PhaseAction phase_name="descend"/>
    <PhaseAction phase_name="approach"/>
</Sequence>
```

**robust_mission.xml** - With fallbacks and retries:
```xml
<Sequence>
    <RetryNode num_attempts="3">
        <PhaseAction phase_name="ascend"/>
    </RetryNode>

    <Fallback name="search_fallback">
        <Sequence>
            <PhaseAction phase_name="scan" params='{"rotation_degrees": 180}'/>
            <HasDetections min_count="1"/>
        </Sequence>
        <Sequence>
            <PhaseAction phase_name="scan" params='{"rotation_degrees": 360}'/>
            <HasDetections min_count="1"/>
        </Sequence>
    </Fallback>

    <!-- ... selection fallbacks, approach with retry ... -->
</Sequence>
```

### 5. Demo Application

Created `demos/05_behavior_tree/bt_mission_runner`:

```bash
# Run behavior tree mission
./bt_mission_runner --tree trees/orange_ball_mission.xml --connect tcp/localhost:7447

# Options:
#   --tree PATH        Path to XML tree file
#   --connect ENDPOINT Zenoh endpoint
#   --robot-id ID      Robot identifier
#   --tick-rate HZ     Tree tick rate (default: 20)
```

### 6. Tests

Added 8 test cases for condition nodes covering:
- Individual condition behavior
- Sequence with multiple conditions
- Fallback logic

All 53 tests pass (45 existing + 8 new).

## Technical Decisions

### No Groot2/ZeroMQ

Groot2 visual editor was intentionally excluded:
- Would require ZeroMQ dependency
- Project already uses Zenoh for all messaging
- Adding a second messaging system would be architectural pollution
- Future: Could build Zenoh-based visualization if needed

### Layer on Top (Not Replace)

Behavior trees orchestrate existing Python phases:
- PhaseAction nodes call phases via Zenoh RPC
- All existing phases work unchanged
- Python-only workflow still works
- BT adds fallbacks/retries/conditions on top

### Blackboard Schema

Used hierarchical keys matching Zenoh conventions:
```
drone/pose/x,y,z,yaw  # From odometry subscription
drone/altitude        # Computed (positive up)
target/selected       # From Python StatePublisher
target/x,y            # Target world position
target/distance       # Computed
inventory/count       # Object count
mission/running       # Mission active flag
```

### Async RPC Pattern

PhaseAction uses non-blocking RPC:
1. `onStart()` - Subscribe to response, publish request
2. `onRunning()` - Check for response (returns RUNNING if waiting)
3. `onHalted()` - Publish cancel request

This allows the BT to remain responsive during long phase execution.

## Files Created

**C++ Library (`libs/behavior_tree/`):**
- `CMakeLists.txt`
- `include/behavior_tree/bt_factory.hpp`
- `include/behavior_tree/bt_conditions.hpp`
- `include/behavior_tree/bt_blackboard_sync.hpp`
- `include/behavior_tree/bt_phase_action.hpp`
- `src/bt_factory.cpp`
- `src/bt_conditions.cpp`
- `src/bt_blackboard_sync.cpp`
- `src/bt_phase_action.cpp`
- `trees/orange_ball_mission.xml`
- `trees/robust_mission.xml`
- `tests/CMakeLists.txt`
- `tests/test_conditions.cpp`

**Python Bridge (`packages/mission_framework/bt_bridge/`):**
- `__init__.py`
- `phase_server.py`
- `state_publisher.py`
- `run_server.py`

**Demo (`demos/05_behavior_tree/`):**
- `CMakeLists.txt`
- `bt_mission_runner.cpp`

**Modified:**
- `CMakeLists.txt` (added behavior_tree lib and demo)

## Usage

```bash
# Terminal 1: Start Python phase server
python -m mission_framework.bt_bridge.run_server --connect tcp/localhost:7447

# Terminal 2: Run behavior tree
./build/linux-release/demos/05_behavior_tree/bt_mission_runner \
    --tree trees/orange_ball_mission.xml \
    --connect tcp/localhost:7447
```

## Benefits

1. **Fallback Strategies**: Try 180° scan, fallback to 360° if no detections
2. **Retry Logic**: Retry failed phases up to N times
3. **Conditional Execution**: Check conditions before proceeding
4. **Parallel Monitoring**: Multiple conditions can run in parallel
5. **XML Configuration**: Change mission logic without recompiling
6. **Preserved Python Workflow**: Existing mission framework unchanged

## Next Steps

1. Test with actual AirSim simulation
2. Add more condition nodes as needed
3. Create more example trees for different mission types
4. Consider Zenoh-based tree visualization tool
5. Add logging/replay capability for debugging
