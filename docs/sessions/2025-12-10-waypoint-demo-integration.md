# Session: Waypoint Demo Integration with AirSim Bridge

**Date:** 2025-12-10
**Focus:** Integrating waypoint following demo with Project AirSim via Zenoh bridge

## Goals

1. Test AirSim bridge with live Project AirSim instance
2. Integrate waypoint following demo with remote AirSim
3. Debug drone commander missions (square, circle, up/down)
4. Investigate drone max velocity limitations
5. Create high-speed drone configurations

## Accomplishments

### 1. Bridge Testing and Debugging

Successfully connected Linux autonomy stack to Windows Project AirSim:

- Fixed Project AirSim API usage (async/await patterns, scene loading)
- Fixed odometry parsing (pose data is dict, not object)
- Fixed depth image parsing (uint16 format, not float32)
- Fixed circle mission (world-frame velocity computation)

### 2. Waypoint Demo Integration

Added `--connect` option to waypoint executables for remote Zenoh connections:

**Modified Files:**
- `demos/02_waypoint_following/WaypointManager.cpp`
- `demos/02_waypoint_following/WaypointPublisher.cpp`

**Usage:**
```bash
# On Linux - waypoint manager connects to Windows bridge
./waypoint_manager --connect tcp/192.168.1.10:7447

# Send waypoints
./waypoint_publisher --connect tcp/192.168.1.10:7447 --pattern square
```

**Test Results:**
```
[Waypoints] Received 4 waypoints
[WP 0] dist=4.40783m vel=(2, 0.275607, 0.0639011)
[Waypoint 0] Reached at distance 0.493845m
[Waypoint 1] Reached at distance 0.490472m
[Waypoint 2] Reached at distance 0.490171m
[Waypoint 3] Reached at distance 0.488046m
[Mission] All waypoints reached!
```

### 3. Drone Commander Tool

Created `drone_commander.py` for interactive and scripted drone control:

- Interactive keyboard control (WASD + Q/E for yaw, R/F for altitude)
- Predefined missions: square, circle, updown
- Single command mode for scripted operations

### 4. Max Velocity Investigation

**Root Cause Analysis:**
- `--max-vel` parameter in waypoint_manager IS parsed correctly
- PID controller IS outputting higher velocities
- **Project AirSim's fast-physics engine limits actual velocity** based on:
  - Rotor thrust (`coeff-of-thrust`, `max-rpm`, `propeller-diameter`)
  - Aerodynamic drag (`drag-coefficient`)

### 5. High-Speed Drone Configurations

Created three performance drone configurations:

| Config | Target Speed | Key Changes |
|--------|--------------|-------------|
| `robot_quadrotor_highspeed_10ms.jsonc` | 10 m/s (36 km/h) | drag: 0.15, thrust: 0.22, rpm: 10000 |
| `robot_quadrotor_highspeed_20ms.jsonc` | 20 m/s (72 km/h) | drag: 0.08, thrust: 0.35, rpm: 15000, prop: 0.28m |
| `robot_quadrotor_highspeed_30ms.jsonc` | 30 m/s (109 km/h) | drag: 0.05, thrust: 0.5, rpm: 25000, prop: 0.32m |

**Physics Parameters:**
- Default: `drag-coefficient: 0.325`, `coeff-of-thrust: 0.109919`, `max-rpm: 6396.667`
- Max velocity ∝ √(thrust/drag)

**Scene Files:**
- `scene_highspeed_10ms.jsonc`
- `scene_highspeed_20ms.jsonc`
- `scene_highspeed_30ms.jsonc`

## Architecture

```
[Windows - Project AirSim]              [Linux - Autonomy Stack]
┌────────────────────────┐              ┌──────────────────────────┐
│  Project AirSim        │              │  waypoint_manager        │
│  (fast-physics sim)    │              │  (PID controller)        │
│         ↕              │              │         ↕                │
│  airsim_zenoh_bridge   │◄─── Zenoh ──►│  waypoint_publisher      │
│  (tcp/0.0.0.0:7447)    │              │  (sends waypoint lists)  │
└────────────────────────┘              └──────────────────────────┘

Data Flow:
  Odometry:  AirSim → Bridge → waypoint_manager
  Waypoints: waypoint_publisher → waypoint_manager
  Velocity:  waypoint_manager → Bridge → AirSim
```

## Issues Encountered

### 1. Project AirSim API Differences
- **Issue:** Incorrect API assumptions from old Microsoft AirSim
- **Solution:** Studied `hello_drone.py` example, used async/await, proper constructors

### 2. Pose Data Format
- **Issue:** `drone.robot_info["actual_pose"]` returns dict, not object
- **Solution:** Parse as `pose_data.get('position', {}).get('x', 0)`

### 3. Depth Image Format
- **Issue:** Depth data is uint16 (2 bytes/pixel), not float32
- **Solution:** Detect format from data size: `if data_size == pixel_count * 2`

### 4. Circle Mission Bug
- **Issue:** Circle mission just flew forward with yaw rate
- **Cause:** World-frame velocity doesn't curve with yaw
- **Solution:** Compute tangential velocity from position relative to circle center

### 5. Simulator Timeout After Failed Scene Load
- **Issue:** `Timeout waiting to get topic info` after loading invalid scene
- **Solution:** Restart Project AirSim completely to clear bad state

## New Files Created

```
sim_interfaces/airsim_zenoh_bridge/
├── drone_commander.py                    # Interactive drone control tool
├── mock_bridge.py                        # Local testing without AirSim
└── sim_config/
    ├── robot_quadrotor_highspeed_10ms.jsonc
    ├── robot_quadrotor_highspeed_20ms.jsonc
    ├── robot_quadrotor_highspeed_30ms.jsonc
    ├── scene_highspeed_10ms.jsonc
    ├── scene_highspeed_20ms.jsonc
    └── scene_highspeed_30ms.jsonc
```

## Commands Reference

### Start Bridge (Windows)
```bash
python airsim_zenoh_bridge.py --scene scene_basic_drone.jsonc --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff

# High-speed drone
python airsim_zenoh_bridge.py --scene scene_highspeed_20ms.jsonc --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff
```

### Run Waypoint Demo (Linux)
```bash
# Terminal 1: PID controller
./waypoint_manager --connect tcp/192.168.1.10:7447 --max-vel 20

# Terminal 2: Send waypoints
./waypoint_publisher --connect tcp/192.168.1.10:7447 --pattern square --size 10
```

### Interactive Control (Linux)
```bash
python drone_commander.py --connect tcp/192.168.1.10:7447

# Or run a mission
python drone_commander.py --connect tcp/192.168.1.10:7447 --mission circle
```

## Next Steps

1. Test high-speed drone configurations (copy files to Windows first)
2. Implement object tracking demo with AirSim bridge
3. Add fish-eye stereo camera configuration (deferred)
4. Investigate PID tuning for high-speed flight
5. Add trajectory smoothing for waypoint transitions
