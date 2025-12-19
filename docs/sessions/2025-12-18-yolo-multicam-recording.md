# Session: YOLO-World Detection, Multi-Camera, and Mission Recording

**Date:** 2025-12-18
**Focus:** Adding DNN-based object detection, multi-camera architecture, and mission recording capabilities

## Goals

1. Replace fragile HSV color detection with YOLO-World zero-shot object detection
2. Design and implement multi-camera detection architecture
3. Create mission recording service for video capture
4. Document mission orchestrator architecture for future development

## Accomplishments

### 1. YOLO-World Object Detection

Replaced error-prone HSV color thresholding with YOLO-World for robust object detection.

**Architecture Decision:** Separate detector node publishing to Zenoh rather than inline detection in mission scripts.

**Files Created:**
- `docs/architecture-yolo-world-detection.md` - Architecture document
- `yolo_world_detector.py` - Single camera detector node
- `fly_to_orange_ball_yolo.py` - Simplified mission using YOLO detections

**Data Types Added to `data_types.py`:**
```python
@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    bbox_x: int
    bbox_y: int
    bbox_w: int
    bbox_h: int
    center_x: int = 0
    center_y: int = 0
    area: int = 0
    bearing_x: float = 0.0  # -1.0 to 1.0 (left to right)
    bearing_y: float = 0.0  # -1.0 to 1.0 (top to bottom)

@dataclass
class DetectionList:
    timestamp_us: int
    frame_id: int
    image_width: int
    image_height: int
    inference_time_ms: float
    detections: List[Detection]

    def get_by_class(self, class_name: str) -> List[Detection]
    def get_best(self) -> Optional[Detection]
    def get_largest(self) -> Optional[Detection]

@dataclass
class DetectorConfig:
    prompts: List[str]
    confidence_threshold: float = 0.25
    nms_threshold: float = 0.45
    max_detections: int = 10
    target_fps: float = 10.0
```

**Zenoh Topics:**
- `robot/{id}/perception/detections` - Published detections
- `robot/{id}/perception/config` - Runtime prompt configuration

### 2. Multi-Camera Detection Architecture

Extended detection to support front, back, and down cameras simultaneously.

**Key Insight:** Front + back cameras enable 180° scan for 360° coverage, halving scan time.

**Files Created:**
- `docs/architecture-multi-camera-detection.md` - 8 capabilities document
- `yolo_multi_camera_detector.py` - Multi-camera YOLO detector
- `fly_to_orange_ball_multicam.py` - Mission with camera handoff

**8 Multi-Camera Capabilities:**
1. Efficient scanning (180° rotation = 360° coverage)
2. Continuous tracking with camera handoff
3. Lost target recovery via rear camera
4. Rear collision safety monitoring
5. Down camera precision for terminal approach
6. Simultaneous search + track
7. Detection triangulation (future)
8. Detection verification across cameras

**Camera Geometry (from robot config):**
```python
DEFAULT_CAMERA_CONFIGS = {
    "front": CameraConfig(position=(0.30, 0.0, -0.05), orientation=(0.0, -15.0, 0.0)),
    "back": CameraConfig(position=(-0.15, 0.0, 0.0), orientation=(0.0, 0.0, 180.0)),
    "down": CameraConfig(position=(0.0, 0.0, 0.05), orientation=(0.0, -90.0, 0.0)),
}
```

**Camera Handoff Logic:**
```python
# Terminal approach: switch from front to down camera
if down_target and dist_xy < 8.0:
    active_camera = "down"
elif front_target:
    active_camera = "front"
```

### 3. Mission Recording Service

Created video recording capability for mission documentation.

**File Created:**
- `record_mission.py` - Mission recording service

**Features:**
- Single camera recording (chase, front, back, down)
- Multi-view layouts: `side_by_side`, `grid` (2x2)
- Auto-layout selection based on camera count
- Configurable FPS, resolution, duration
- H.264 conversion via ffmpeg if available
- Timestamp overlay

**Usage:**
```bash
# Chase camera (cinematic view)
python record_mission.py --connect tcp/192.168.1.10:7447 --camera chase

# 2x2 grid of all cameras
python record_mission.py --connect tcp/192.168.1.10:7447 \
    --cameras front back down chase --layout grid

# Side-by-side with duration limit
python record_mission.py --connect tcp/192.168.1.10:7447 \
    --cameras chase front --layout side_by_side --duration 120
```

### 4. Chase Camera Bridge Support

Added external chase camera to the AirSim bridge.

**Change in `airsim_zenoh_bridge.py`:**
```python
camera_name_map = {
    "FrontCamera": "front",
    "BackCamera": "back",
    "DownCamera": "down",
    "Chase": "chase",  # External 3rd-person chase camera
}
```

**Note:** Chase camera must be defined in robot config (e.g., `robot_quadrotor_vision.jsonc`).

### 5. Mission Orchestrator Architecture

Documented comprehensive architecture for mission management.

**File Created:**
- `docs/architecture-mission-orchestrator.md`

**Key Sections:**
1. **YAML Configuration Schema** - Declarative mission definition
2. **Phase Library** - Reusable mission phases (Search, Track, Approach, Land)
3. **Recording Service Integration** - Video capture coordination
4. **Behavior Trees Path** - When/how to adopt BTs
5. **LLM/VLM Planning Research** - Survey of current approaches:
   - Behavior Trees: Mature, composable, deterministic
   - LLM Planning: SayCan, PaLM-E, Code as Policies
   - Hybrid LLM+BT: 2024 trend combining reasoning with robustness
6. **VLA Research** - Vision-Language-Action for drones:
   - UAV-VLA (2024): VLM → action tokens for navigation
   - CognitiveDrone: Three-tier architecture
   - Key challenge: Sim-to-real transfer

**Evolution Roadmap:**
```
YAML Config → Behavior Trees → LLM+BT Hybrid
    (now)        (next)          (future)
```

## Technical Decisions

### Why Separate Detector Node?
- Reusable across missions
- Consistent detection interface
- GPU resource management
- Runtime prompt changes without mission restart

### Why YOLO-World over Standard YOLO?
- Zero-shot detection via text prompts
- No training required for new objects
- Change prompts at runtime: `["orange ball"]` → `["person", "vehicle"]`

### BT vs LLM for Planning?
- **Not mutually exclusive** - 2024 research shows hybrids work best
- LLMs excel at: Goal decomposition, natural language, adaptation
- BTs excel at: Real-time execution, safety, determinism
- **Recommendation:** Start with YAML/BT, add LLM layer later

## Files Modified

- `airsim_zenoh_bridge.py` - Added multi-camera topics, chase camera support
- `data_types.py` - Added Detection, DetectionList, CameraDetectionList, DetectorConfig

## Files Created

- `docs/architecture-yolo-world-detection.md`
- `docs/architecture-multi-camera-detection.md`
- `docs/architecture-mission-orchestrator.md`
- `yolo_world_detector.py`
- `yolo_multi_camera_detector.py`
- `fly_to_orange_ball_yolo.py`
- `fly_to_orange_ball_multicam.py`
- `record_mission.py`

## Next Steps

1. Test recording service with actual mission flight
2. Implement Phase Library for mission orchestrator
3. Create YAML config loader and mission runner
4. Add BT integration when mission complexity warrants it
