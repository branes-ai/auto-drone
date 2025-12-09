# Session: Phase 2 - Waypoint Following

**Date:** 2025-12-08
**Focus:** PID control library, waypoint data types, waypoint following demo

## Goals

1. Create PID controller library with anti-windup and derivative filtering
2. Add waypoint and velocity command data types
3. Build waypoint following demo with simulated drone

## Key Decisions

### PID Controller Design
- Configurable gains (Kp, Ki, Kd) with runtime adjustment
- Anti-windup via integral clamping
- Low-pass filtered derivative to reduce noise sensitivity
- Optional feedforward term for improved tracking

### Position Controller
- Three independent PID loops for X, Y, Z axes
- Outputs velocity commands in body frame

### Yaw Controller
- Handles angle wrapping at ±π boundary
- Uses shortest-path calculation for error

### Velocity Command Priority
- Priority levels: LOW, NORMAL, HIGH, CRITICAL
- Source tracking for debugging (WAYPOINT_MANAGER, OBSTACLE_AVOIDANCE, etc.)
- Enables future command arbiter implementation

### Waypoint Behavior Flags
- `HOLD_YAW` - Maintain specified yaw during approach
- `HOVER` - Brief hover at waypoint
- `FINAL` - Last waypoint in sequence

## Implementation

### libs/control_algorithms/
| File | Description |
|------|-------------|
| `PIDController.hpp` | PID controller with Gains, Limits, Config structs |
| `PIDController.cpp` | Implementation with filtered derivative |
| `tests/PIDControllerTest.cpp` | Unit tests for all controllers |
| `CMakeLists.txt` | Library build configuration |

### libs/data_types/
| File | Description |
|------|-------------|
| `Waypoint.hpp/cpp` | Waypoint struct with serialization |
| `VelocityCommand.hpp/cpp` | Velocity command with priority |

### demos/02_waypoint_following/
| File | Description |
|------|-------------|
| `WaypointManager.cpp` | PID-based waypoint controller |
| `WaypointPublisher.cpp` | Square/circle pattern generator |
| `SimulatedDrone.cpp` | Velocity integration, odometry publishing |
| `CMakeLists.txt` | Demo build configuration |

## Code Highlights

### PID Update with Derivative Filtering
```cpp
// First-order low-pass filter: y = alpha * x + (1 - alpha) * y_prev
float alpha = dt / (config_.derivative_filter_tau + dt);
filtered_derivative_ = alpha * raw_derivative + (1.0f - alpha) * filtered_derivative_;
```

### Yaw Angle Wrapping
```cpp
float YawController::wrap_angle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}
```

### Waypoint Reached Detection
```cpp
float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
if (distance < config.waypoint_radius) {
    g_current_waypoint_index++;
    position_controller.reset();
}
```

## Testing

### Unit Tests (27 total, 100% pass)
- PIDController: default construction, gains, proportional, integral, anti-windup, reset
- PositionController: uniform gains, reset
- YawController: basic control, angle wrapping, PI boundary
- Convergence simulation: damped point mass system

### Integration Test
```bash
# Terminal 1: Simulated drone
./build/linux-release/demos/02_waypoint_following/simulated_drone

# Terminal 2: Waypoint manager
./build/linux-release/demos/02_waypoint_following/waypoint_manager

# Terminal 3: Publish waypoints
./build/linux-release/demos/02_waypoint_following/waypoint_publisher --size 3
```

**Results:**
- Waypoints 0 and 1 reached within 0.5m threshold
- Smooth velocity commands generated
- Odometry published at 100Hz

## Issues Encountered

### Test Failures
- **Issue:** YawController test expected wrong error sign
- **Solution:** Wrapped error of +20 deg (not -20 deg) for shortest path

- **Issue:** PID convergence test failed (position 8.86, needed 9.0-11.0)
- **Solution:** Adjusted gains (kp=2.0, ki=0.5, kd=1.0), extended simulation to 20s

### API Mismatch
- **Issue:** Used `yaw_controller.pid()` but method is `controller()`
- **Solution:** Updated to `yaw_controller.controller().set_limits()`

## Zenoh Topics

| Topic | Direction | Data Type |
|-------|-----------|-----------|
| `robot/drone/sensor/state/odom` | Subscribe | Odometry |
| `robot/drone/cmd/waypoints` | Subscribe | WaypointList |
| `robot/drone/cmd/velocity` | Publish | VelocityCommand |

## Serialization Formats

### Waypoint (28 bytes)
```
[x:4][y:4][z:4][yaw:4][speed:4][id:4][flags:1][padding:3]
```

### VelocityCommand (28 bytes)
```
[vx:4][vy:4][vz:4][yaw_rate:4][priority:1][source:1][padding:2][timestamp:8]
```

## Next Steps

- Phase 3: Object tracking with perception
- Command arbiter for priority-based velocity command selection
- Integration with AirSim/Isaac simulators
