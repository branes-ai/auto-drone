# Session: Mission Framework Implementation

**Date:** 2025-12-19
**Focus:** Implementing the Phase Library for mission orchestration

## Goals

1. Implement the Phase Library designed in the architecture document
2. Create reusable phases for drone missions
3. Enable YAML-based mission configuration
4. Eliminate code duplication across mission scripts

## Accomplishments

### 1. Mission Framework Package

Created `packages/mission_framework/` - a reusable Python package for drone mission orchestration.

**Package Structure:**
```
packages/mission_framework/
├── __init__.py
├── config.py           # MissionConfig, PhaseConfig, YAML loading
├── state.py            # MissionState, TrackedObject, ObjectInventory
├── runner.py           # MissionRunner orchestrator
├── phases/
│   ├── __init__.py     # Phase registry with @register_phase decorator
│   ├── base.py         # MissionPhase ABC, PhaseResult, PhaseStatus
│   ├── ascend.py       # Rise to observation altitude
│   ├── scan.py         # 180°/360° rotation with detection
│   ├── select.py       # Choose best target from inventory
│   ├── navigate.py     # Cruise to target position
│   ├── descend.py      # Drop to approach altitude
│   ├── approach.py     # Visual servoing with camera handoff
│   └── land.py         # Landing + hover phases
├── services/
│   ├── __init__.py
│   ├── drone.py        # DroneService (odometry, velocity)
│   └── detection.py    # DetectionService (multi-camera)
└── configs/
    └── orange_ball.yaml
```

### 2. Core Components

#### MissionState (`state.py`)
Thread-safe shared state for mission execution:
- `TrackedObject` - Detection with world position estimation
- `ObjectInventory` - Collection with duplicate merging (10m threshold)
- Pose, target, and phase data management

#### MissionConfig (`config.py`)
YAML-based configuration:
```yaml
name: orange_ball_multicam
cameras: [front, back, down]
target_class: "orange ball"
observation_altitude: 25.0
phases:
  - type: ascend
  - type: scan
    params:
      rotation_degrees: 180
  - type: select
  - type: navigate
  - type: descend
  - type: approach
```

#### MissionRunner (`runner.py`)
Async orchestrator:
- Zenoh connection management
- Service lifecycle (start/stop)
- Phase instantiation from registry
- Timeout and cancellation handling

### 3. Services

#### DroneService
- Subscribes to odometry topic
- Publishes velocity commands (world frame NED)
- Provides pose access and hover/emergency_stop helpers

#### DetectionService
- Subscribes to per-camera detection topics
- Aggregates detections from multiple cameras
- Camera geometry for world position estimation
- `get_best_detection()` across all cameras

### 4. Phase Implementations

| Phase | Description |
|-------|-------------|
| `ascend` | Rise to observation altitude with P-control |
| `scan` | Rotate 180°/360° while collecting detections |
| `select` | Choose best target (confidence/size/distance) |
| `navigate` | Cruise to target at altitude with yaw tracking |
| `descend` | Drop to approach altitude, hold XY position |
| `approach` | Visual servoing with front→down camera handoff |
| `hover` | Hold position for specified duration |
| `land` | Controlled descent to ground |

### 5. CLI Entry Point

`run_mission.py` in `sim_interfaces/airsim_zenoh_bridge/`:

```bash
# Run with config
python run_mission.py --connect tcp/localhost:7447 --config orange_ball.yaml

# With overrides
python run_mission.py --connect tcp/localhost:7447 \
    --config orange_ball.yaml --altitude 30 --target "red cube"

# Default mission (no config)
python run_mission.py --connect tcp/localhost:7447

# List phases
python run_mission.py --list-phases
```

### 6. Recording Service Chase Camera

Added chase camera support to AirSim bridge:
```python
camera_name_map = {
    "FrontCamera": "front",
    "BackCamera": "back",
    "DownCamera": "down",
    "Chase": "chase",  # External 3rd-person view
}
```

## Technical Decisions

### Package Location
Initially placed under `sim_interfaces/airsim_zenoh_bridge/missions/`, but moved to `packages/mission_framework/` because:
- The Phase Library is simulator-agnostic
- Should be a reusable library, not tied to AirSim
- `packages/` is appropriate for Python packages (vs `libs/` for C++)

### Async Design
All phases and services use asyncio:
- Non-blocking waits for sensor data
- Proper cancellation support
- Compatible with Zenoh callbacks

### Thread Safety
MissionState uses locks around all mutable data:
- Zenoh callbacks run in separate threads
- Phases read state from main async loop
- Lock-protected properties for safe access

### Phase Registry Pattern
```python
@register_phase("ascend")
class AscendPhase(MissionPhase):
    async def execute(self) -> PhaseResult:
        ...
```
- Phases self-register with decorator
- YAML configs reference by string type name
- Easy to add new phases

### Import Strategy
Services use try/except for data_types import:
```python
try:
    from data_types import Odometry, VelocityCommand
except ImportError:
    # Fallback path resolution
```
This allows running from different working directories.

## Files Created

**Package:**
- `packages/mission_framework/__init__.py`
- `packages/mission_framework/config.py`
- `packages/mission_framework/state.py`
- `packages/mission_framework/runner.py`
- `packages/mission_framework/phases/__init__.py`
- `packages/mission_framework/phases/base.py`
- `packages/mission_framework/phases/ascend.py`
- `packages/mission_framework/phases/scan.py`
- `packages/mission_framework/phases/select.py`
- `packages/mission_framework/phases/navigate.py`
- `packages/mission_framework/phases/descend.py`
- `packages/mission_framework/phases/approach.py`
- `packages/mission_framework/phases/land.py`
- `packages/mission_framework/services/__init__.py`
- `packages/mission_framework/services/drone.py`
- `packages/mission_framework/services/detection.py`
- `packages/mission_framework/configs/orange_ball.yaml`

**CLI:**
- `sim_interfaces/airsim_zenoh_bridge/run_mission.py` (updated)

**Bridge:**
- `sim_interfaces/airsim_zenoh_bridge/airsim_zenoh_bridge.py` (chase camera)

## Expected Benefits

1. **Code Reduction**: ~65% less code per mission (700 lines → 50 lines YAML)
2. **Reusability**: Phases work across different mission types
3. **Configuration**: Parameter tuning without code changes
4. **Testability**: Individual phases can be unit tested
5. **Extensibility**: Easy to add new phases
6. **Foundation**: Ready for Behavior Tree integration

## Next Steps

1. Test the framework with actual missions
2. Migrate existing `fly_to_orange_ball_*.py` scripts to use framework
3. Add more phases as needed (waypoint, patrol, return-to-home)
4. Consider adding recording service integration
5. Implement Behavior Tree layer when mission complexity warrants
