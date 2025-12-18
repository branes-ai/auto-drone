# Mission Orchestrator Architecture

This document describes the architecture for a mission orchestration system that enables declarative mission definitions, reusable phases, and recording capabilities. It also explores the path from simple orchestration to behavior trees and LLM/VLA-based adaptive planning.

## 1. Current State: Problem Statement

We have accumulated multiple mission scripts with significant code duplication:

```
fly_to_orange_ball.py          # Original HSV-based
fly_to_orange_ball_phased.py   # Phased approach
fly_to_orange_ball_yolo.py     # YOLO-based detection
fly_to_orange_ball_multicam.py # Multi-camera support
```

Each script reimplements:
- Zenoh connection and subscription
- Odometry handling
- Velocity command sending
- Phase execution logic
- Detection processing

**Goals:**
1. Eliminate code duplication
2. Enable rapid mission prototyping via configuration
3. Support recording/playback
4. Provide a path to more sophisticated planning

## 2. Proposed Architecture

### 2.1 System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Mission Configuration                            │
│                         (YAML or JSONC file)                            │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          Mission Runner                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │ Config      │  │ Phase       │  │ State       │  │ Event       │    │
│  │ Loader      │  │ Executor    │  │ Manager     │  │ Bus         │    │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
         │                   │                   │                │
         ▼                   ▼                   ▼                ▼
┌─────────────┐    ┌─────────────────────────────────────┐    ┌─────────┐
│ Phase       │    │         Core Services               │    │Recording│
│ Library     │    │  ┌─────────┐ ┌─────────┐ ┌───────┐ │    │ Service │
│ ┌─────────┐ │    │  │ Drone   │ │Detection│ │ Odom  │ │    │         │
│ │ Ascend  │ │    │  │ Control │ │ Service │ │Service│ │    │         │
│ │ Scan    │ │    │  └─────────┘ └─────────┘ └───────┘ │    │         │
│ │ Select  │ │    └─────────────────────────────────────┘    └─────────┘
│ │Navigate │ │                      │
│ │Approach │ │                      ▼
│ └─────────┘ │              ┌─────────────┐
└─────────────┘              │   Zenoh     │
                             │   Network   │
                             └─────────────┘
```

### 2.2 File Structure

```
sim_interfaces/airsim_zenoh_bridge/
├── missions/
│   ├── __init__.py
│   ├── runner.py                 # Mission runner/orchestrator
│   ├── config.py                 # Config schema and loader
│   ├── state.py                  # Mission state management
│   │
│   ├── phases/                   # Reusable phase implementations
│   │   ├── __init__.py
│   │   ├── base.py              # MissionPhase abstract base
│   │   ├── ascend.py
│   │   ├── scan.py
│   │   ├── select.py
│   │   ├── navigate.py
│   │   ├── approach.py
│   │   └── land.py
│   │
│   ├── services/                 # Background services
│   │   ├── __init__.py
│   │   ├── drone.py             # Drone control (velocity, pose)
│   │   ├── detection.py         # Detection aggregation
│   │   └── recorder.py          # Video recording service
│   │
│   └── configs/                  # Mission definitions
│       ├── orange_ball.yaml
│       ├── orange_ball_multicam.yaml
│       ├── inspection.yaml
│       └── search_rescue.yaml
│
├── run_mission.py               # CLI entry point
└── record_mission.py            # Recording CLI
```

## 3. Mission Configuration Schema

### 3.1 YAML Configuration Example

```yaml
# missions/configs/orange_ball_multicam.yaml
mission:
  name: "Find Orange Ball - Multi-Camera"
  version: "1.0"
  description: "Locate and approach orange ball using front+back+down cameras"

# Target specification
target:
  class: "orange ball"
  aliases: ["orange sphere", "ball"]  # Alternative detection names

# Camera configuration
cameras:
  front:
    enabled: true
    prompts: ["orange ball", "obstacle"]
    role: navigation
  back:
    enabled: true
    prompts: ["orange ball"]
    role: scanning
  down:
    enabled: true
    prompts: ["orange ball"]
    role: precision

# Mission phases (executed in order)
phases:
  - name: ascend
    params:
      altitude: 25.0        # meters
      speed: 3.0            # m/s

  - name: scan
    params:
      rotation: 180         # degrees (180 with front+back = 360 coverage)
      yaw_rate: 0.5         # rad/s
      cameras: [front, back]

  - name: select
    params:
      criteria: highest_confidence
      min_confidence: 0.3
      # Alternative criteria: largest_area, closest, highest_score

  - name: navigate
    params:
      strategy: fly_over_then_descend
      cruise_altitude: 25.0
      descent_threshold: 10.0  # Start descent when within this XY distance
      speed: 5.0

  - name: descend
    params:
      target_altitude: 5.0
      speed: 2.0

  - name: approach
    params:
      cameras: [front, down]
      handoff_distance: 8.0   # Switch to down camera within this distance
      success_distance: 3.0
      timeout: 60.0

# Recording configuration (optional)
recording:
  enabled: true
  cameras: [chase]           # Which cameras to record
  output: "mission_${timestamp}.mp4"
  fps: 30
  layout: single             # single, grid_2x2, side_by_side

# Safety parameters
safety:
  min_altitude: 2.0
  max_altitude: 50.0
  geofence:
    enabled: false
    radius: 100.0
    center: [0, 0]
```

### 3.2 CLI Override Examples

```bash
# Run mission with config
python run_mission.py --config missions/configs/orange_ball_multicam.yaml \
    --connect tcp/192.168.1.10:7447

# Override target
python run_mission.py --config orange_ball.yaml --target "red cone"

# Override altitude
python run_mission.py --config orange_ball.yaml --altitude 30

# Enable recording
python run_mission.py --config orange_ball.yaml --record --record-cameras chase front

# Dry run (validate config without executing)
python run_mission.py --config orange_ball.yaml --dry-run
```

## 4. Phase Library Design

### 4.1 Base Phase Class

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict, Any
from enum import Enum

class PhaseStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    ABORTED = "aborted"

@dataclass
class PhaseResult:
    status: PhaseStatus
    message: str = ""
    data: Dict[str, Any] = None  # Phase-specific output data

class MissionPhase(ABC):
    """Base class for all mission phases."""

    def __init__(self, name: str, params: Dict[str, Any], services: 'ServiceContainer'):
        self.name = name
        self.params = params
        self.services = services
        self.status = PhaseStatus.PENDING

    @abstractmethod
    async def execute(self) -> PhaseResult:
        """Execute the phase. Returns when phase completes or fails."""
        pass

    def abort(self):
        """Request phase abortion."""
        self.status = PhaseStatus.ABORTED

    @property
    def drone(self) -> 'DroneService':
        return self.services.drone

    @property
    def detection(self) -> 'DetectionService':
        return self.services.detection
```

### 4.2 Example Phase Implementation

```python
# missions/phases/ascend.py
class AscendPhase(MissionPhase):
    """Ascend to target altitude."""

    async def execute(self) -> PhaseResult:
        altitude = self.params.get('altitude', 25.0)
        speed = self.params.get('speed', 3.0)

        print(f"[ASCEND] Rising to {altitude}m...")

        while self.status != PhaseStatus.ABORTED:
            pose = await self.drone.get_pose()
            current_alt = -pose.z  # NED to altitude

            if current_alt >= altitude - 0.5:
                await self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.SUCCEEDED,
                    message=f"Reached {current_alt:.1f}m",
                    data={'final_altitude': current_alt}
                )

            await self.drone.send_velocity(0, 0, -speed, 0)
            await asyncio.sleep(0.1)

        return PhaseResult(status=PhaseStatus.ABORTED)
```

### 4.3 Phase Registry

```python
# missions/phases/__init__.py
from .ascend import AscendPhase
from .scan import ScanPhase
from .select import SelectPhase
from .navigate import NavigatePhase
from .approach import ApproachPhase

PHASE_REGISTRY = {
    'ascend': AscendPhase,
    'scan': ScanPhase,
    'select': SelectPhase,
    'navigate': NavigatePhase,
    'approach': ApproachPhase,
    'descend': DescendPhase,
    'land': LandPhase,
    'hover': HoverPhase,
    'wait': WaitPhase,
}

def create_phase(name: str, params: dict, services) -> MissionPhase:
    if name not in PHASE_REGISTRY:
        raise ValueError(f"Unknown phase: {name}")
    return PHASE_REGISTRY[name](name, params, services)
```

## 5. Recording Service

### 5.1 Design

The recording service runs independently and can record any mission:

```python
class RecordingService:
    """Records camera feeds to video file."""

    def __init__(self, config: RecordingConfig):
        self.cameras = config.cameras      # ['chase', 'front']
        self.output = config.output        # 'mission.mp4'
        self.fps = config.fps              # 30
        self.layout = config.layout        # 'single', 'grid_2x2'

        self.frames = {cam: [] for cam in self.cameras}
        self.recording = False

    async def start(self):
        """Start recording."""
        self.recording = True
        # Subscribe to camera topics
        for camera in self.cameras:
            topic = f"robot/drone/sensor/camera/{camera}/rgb"
            self.session.declare_subscriber(topic, self._on_frame)

    async def stop(self):
        """Stop recording and save video."""
        self.recording = False
        await self._save_video()

    def _on_frame(self, sample, camera_id):
        if self.recording:
            frame = ImageData.deserialize(sample.payload).to_numpy()
            self.frames[camera_id].append((time.time(), frame))

    async def _save_video(self):
        """Compose frames and save using ffmpeg."""
        # Combine frames based on layout
        # Write to video file
```

### 5.2 Recording Layouts

```
Single Camera:          Grid 2x2:              Side-by-Side:
┌─────────────────┐    ┌────────┬────────┐    ┌──────────┬──────────┐
│                 │    │ Front  │  Back  │    │  Chase   │  Front   │
│     Chase       │    ├────────┼────────┤    │          │          │
│                 │    │  Down  │ Chase  │    │          │          │
└─────────────────┘    └────────┴────────┘    └──────────┴──────────┘
```

## 6. Path to Behavior Trees

### 6.1 What Are Behavior Trees?

Behavior Trees (BTs) originated in game AI (Halo 2, ~2004) and were adopted by robotics around 2010. They provide:

- **Modularity**: Behaviors are composable nodes
- **Reactivity**: Can respond to changing conditions
- **Hierarchy**: Complex behaviors from simple primitives
- **Debuggability**: Visual tree structure easy to understand

```
         [Sequence]
        /     |     \
   [Ascend] [Scan] [Fallback]
                   /        \
            [Approach]  [SearchMore]
```

### 6.2 BT Node Types

| Node Type | Symbol | Behavior |
|-----------|--------|----------|
| Sequence | → | Execute children left-to-right, fail if any fails |
| Fallback | ? | Try children until one succeeds |
| Parallel | ⇉ | Execute children simultaneously |
| Decorator | ◇ | Modify child behavior (retry, timeout, invert) |
| Action | □ | Leaf node that does something |
| Condition | ○ | Leaf node that checks something |

### 6.3 Our Phases as BT

Our current phase-based approach maps naturally to a BT Sequence:

```
                    [Sequence: OrangeBallMission]
                    /    |      |       |        \
            [Ascend] [Scan] [Select] [Navigate] [Approach]
               │       │       │         │          │
            action  action  action    action     [Fallback]
                                                 /        \
                                          [VisualApproach] [WaypointApproach]
```

### 6.4 When to Upgrade to BTs?

**Current Phase System is Sufficient When:**
- Missions are linear sequences
- Failure handling is simple (abort mission)
- No complex conditional logic

**Upgrade to BTs When You Need:**
- Dynamic replanning (target lost → search → reacquire)
- Parallel behaviors (scan while navigating)
- Complex fallback strategies
- Multi-agent coordination
- Runtime behavior modification

### 6.5 BT Implementation Options

| Library | Language | Notes |
|---------|----------|-------|
| [py_trees](https://py-trees.readthedocs.io/) | Python | Mature, well-documented |
| [BehaviorTree.CPP](https://www.behaviortree.dev/) | C++ | ROS2 standard, very fast |
| [Groot](https://www.behaviortree.dev/groot/) | GUI | Visual BT editor |

## 7. LLM/VLM-Based Planning: The New Frontier

### 7.1 Are Behavior Trees Obsolete?

**No.** The 2024 research consensus is that BTs and LLMs are **complementary**:

> "LLM-based methods, while demonstrating strong reasoning capabilities, often generate task plans in a single-shot manner and lack mechanisms for online correction once execution begins. This makes them fragile in dynamic or partially observable environments."
>
> — [LLM-HBT: Dynamic Behavior Tree Construction](https://arxiv.org/html/2510.09963)

**Emerging Hybrid Approaches (2024):**

1. **[LLM-BT](https://arxiv.org/html/2404.05134v1)**: LLM generates and modifies BTs at runtime based on environmental changes

2. **[BTGenBot](https://arxiv.org/html/2403.12761v1)**: Uses smaller 7B parameter models to generate BTs locally, without cloud APIs

3. **[LLM-as-BT-Planner](https://arxiv.org/abs/2409.10444)**: LLM generates complete BTs from natural language task descriptions

### 7.2 Comparison: BT vs LLM vs Hybrid

| Aspect | Behavior Trees | Pure LLM | Hybrid (LLM+BT) |
|--------|---------------|----------|-----------------|
| **Latency** | <1ms | 100ms-2s | Variable |
| **Determinism** | Fully deterministic | Non-deterministic | Controllable |
| **Novel situations** | Cannot handle | Excellent | Excellent |
| **Reliability** | Very high | Medium | High |
| **Explainability** | Excellent (visual) | Poor (black box) | Good |
| **Offline operation** | Yes | Needs connectivity* | Partial |
| **Compute requirements** | Minimal | GPU/Cloud | Moderate |

*Unless using local models like Llama

### 7.3 VLA Models for Drones: State of the Art

**Vision-Language-Action (VLA)** models directly output control actions from visual input and language instructions. Recent drone-specific research:

#### UAV-VLA (January 2025)
[Paper](https://arxiv.org/abs/2501.05014) | [Details](https://www.aimodels.fyi/papers/arxiv/uav-vla-vision-language-action-system-large)

- Integrates satellite imagery + VLM + GPT for mission planning
- Generates flight paths from natural language: *"Survey the northern farmland for irrigation issues"*
- **6.5x faster** than human operators for mission planning
- Creates 100K example dataset for training

```
User: "Inspect the three water towers in the industrial zone"
     │
     ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ Goal Extraction │ ──▶ │ Object Search   │ ──▶ │ Action          │
│ (GPT)           │     │ (VLM + Satellite│     │ Generation      │
│                 │     │  Imagery)       │     │ (GPT)           │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                        │
                                                        ▼
                                               Flight Plan + Actions
```

#### UAV-VLPA* (March 2025)
[Paper](https://arxiv.org/html/2503.02454v1)

- Extends UAV-VLA with **A* path planning** for obstacle avoidance
- Solves **Traveling Salesman Problem** for multi-waypoint optimization
- **18.5% shorter trajectories** than baseline methods

#### CognitiveDrone (2024)
[Project](https://cognitivedrone.github.io/)

- VLA model outputting **4D action commands** (vx, vy, vz, yaw_rate)
- Trained on 8,000+ simulated trajectories
- Tasks: Human Recognition, Symbol Understanding, Reasoning
- **77.2% success rate** on cognitive tasks (CognitiveDrone-R1)

### 7.4 When to Consider VLA?

**VLA is Appropriate When:**
- Missions are described in natural language
- Environment is unstructured/unknown
- Tasks require semantic understanding ("find the person in red")
- Rapid adaptation to novel situations needed

**VLA is NOT Ready For:**
- Safety-critical real-time control (latency too high)
- Deterministic, repeatable missions
- Offline/edge deployment (compute requirements)
- Regulatory compliance (explainability requirements)

## 8. Recommended Evolution Path

```
Current State          Phase 1              Phase 2              Phase 3
─────────────────────────────────────────────────────────────────────────────
Loose Scripts    →    YAML Config     →    Behavior Trees   →   LLM+BT Hybrid
                      + Phase Library      + py_trees            + Local VLM
                      + Recording

Complexity: Low       Medium               Medium-High          High
Flexibility: Low      Medium               High                 Very High
Time to MVP: Done     2-3 days             1-2 weeks            Ongoing R&D
```

### Phase 1: YAML Config + Phase Library (Immediate)
- Eliminate code duplication
- Enable rapid prototyping
- Add recording capability
- **Deliverables**: `run_mission.py`, phase library, recording service

### Phase 2: Behavior Trees (When Needed)
- Implement when missions require:
  - Fallback strategies
  - Parallel execution
  - Dynamic replanning
- **Tool**: py_trees + Groot for visualization

### Phase 3: LLM Integration (Future)
- Start with LLM-generated mission configs (not runtime control)
- Example: "Create a mission to find and photograph all red objects"
- LLM outputs YAML config, human reviews, system executes
- Later: Runtime BT modification via LLM

### Phase 4: VLA Exploration (Research)
- Monitor UAV-VLA, CognitiveDrone progress
- Experiment with local VLMs (LLaVA, Qwen-VL)
- Consider for high-level planning, not low-level control

## 9. Implementation Priority

| Priority | Component | Effort | Value |
|----------|-----------|--------|-------|
| 1 | Recording Service | Low | High (immediate demo value) |
| 2 | Phase Base Class | Low | High (foundation) |
| 3 | Config Loader | Medium | High (enables YAML missions) |
| 4 | Mission Runner | Medium | High (ties it together) |
| 5 | Refactor existing phases | Medium | Medium (reduces duplication) |
| 6 | BT Migration | High | Medium (when needed) |
| 7 | LLM Config Generation | Medium | Low (nice-to-have) |

## 10. Next Steps

1. **Implement Recording Service** (this session)
   - Single camera and multi-view support
   - ffmpeg-based video generation

2. **Create Phase Base Class**
   - Abstract interface for all phases
   - Service container for dependency injection

3. **Implement Config Loader**
   - YAML parsing with validation
   - CLI override support

4. **Build Mission Runner**
   - Phase instantiation and execution
   - Event logging for debugging

## References

### Behavior Trees
- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)

### LLM + Robotics
- [LLM-BT: Adaptive Tasks with LLMs and Behavior Trees](https://arxiv.org/html/2404.05134v1)
- [BTGenBot: BT Generation with Lightweight LLMs](https://arxiv.org/html/2403.12761v1)
- [LLM-as-BT-Planner](https://arxiv.org/abs/2409.10444)

### VLA for Drones
- [UAV-VLA: Vision-Language-Action for Aerial Missions](https://arxiv.org/abs/2501.05014)
- [UAV-VLPA*: VLA with Path Planning](https://arxiv.org/html/2503.02454v1)
- [CognitiveDrone: VLA Model & Benchmark](https://cognitivedrone.github.io/)
