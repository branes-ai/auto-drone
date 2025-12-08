# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Zenoh-based R&D environment for autonomous drone/robot development**. It explicitly avoids ROS/ROS 2 in favor of raw Linux and Zenoh for high-performance, decentralized data communication. The system interfaces with IAMAI Project AirSim and NVIDIA Isaac for simulation.

## Build System

### Prerequisites

- CMake 3.21+ (for presets)
- C++20 compiler (GCC 11+, Clang 14+, MSVC 2022)
- Ninja build system
- Zenoh-C library
- OpenCV 4.x

### Installing Dependencies

See `docs/INSTALL.md` for detailed instructions, or use the install scripts:

```bash
# Linux
./scripts/install-deps-linux.sh

# macOS
./scripts/install-deps-macos.sh

# Windows (PowerShell as Administrator)
.\scripts\install-deps-windows.ps1
```

### Building with CMake Presets

```bash
# Linux
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release

# macOS
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release

# Windows (Visual Studio)
cmake --preset windows-release
cmake --build --preset windows-release
ctest --preset windows-release

# Windows (Ninja from VS Developer Command Prompt)
cmake --preset windows-ninja-release
cmake --build --preset windows-ninja-release
```

### Running the Demo

```bash
# Terminal 1: Start the mock publisher
./build/linux-release/demos/01_sensor_streaming/mock_publisher

# Terminal 2: Start the viewer
./build/linux-release/demos/01_sensor_streaming/viewer_node
```

## Architecture

### Core Design Principle
Zenoh pub/sub primitives form the communication backbone. All components communicate via Zenoh key expressions (e.g., `robot/drone/sensor/camera/rgb`, `robot/drone/cmd/velocity`).

### Directory Structure
```
libs/                    # Core libraries
  zenoh_interface/       # Zenoh pub/sub wrappers
  data_types/            # Serialization and data structures
  control_algorithms/    # PID, trajectory generation

sim_interfaces/          # Simulation connectors
  airsim_client/         # AirSim API -> Zenoh bridge
  isaac_client/          # NVIDIA Isaac -> Zenoh bridge

platforms/               # Robot-specific integration
  drone_platform/        # Drone vehicle interface

autonomy_stack/          # High-level autonomous behaviors
  visual_slam/           # ORB-SLAM3 integration
  object_tracking/       # Perception/tracking
  visual_nav/            # Path planning

demos/                   # Demonstration executables
  01_sensor_streaming/
  02_object_tracking/
  03_obstacle_avoidance/
```

### Zenoh Key Expression Conventions
- Sensor data: `robot/{id}/sensor/{type}` (e.g., `robot/drone/sensor/camera/rgb`, `robot/drone/sensor/imu`)
- Commands: `robot/{id}/cmd/{type}` (e.g., `robot/drone/cmd/velocity`)
- State: `robot/{id}/state/{type}` (e.g., `robot/drone/state/pose_slam`)
- Swarm coordination: `swarm/coordinator/task_assignment`
- Multi-robot queries use wildcards: `robot/*/state/pose_slam`

### Development Phases
1. **Communication Core**: Zenoh interface, simulator clients, sensor streaming
2. **Basic Autonomy**: Control algorithms, waypoint following
3. **Perceptive Autonomy**: Object tracking, obstacle avoidance
4. **Advanced Autonomy**: Visual SLAM, visual waypoint navigation
5. **Multi-Agent**: Swarm coordination, ML inference nodes

## Code Style

- **File extensions**: Use `.hpp`/`.cpp` for C++ code, `.h`/`.c` for C code
- This distinction clarifies C vs C++ boundaries, especially important when interfacing with Zenoh-C

## Key Implementation Notes

- **Shared Memory**: When simulator and autonomy nodes run on the same machine, Zenoh uses SHM automatically for large payloads (critical for real-time SLAM with image streams)
- **Time Synchronization**: SLAM nodes require synchronized, timestamped sensor data (camera + IMU)
- **Command Arbiter Pattern**: Multiple command sources (nominal, reactive, ML) feed into an arbiter that selects final commands
- **C++ for performance-critical paths**: Use C++/Zenoh for SLAM, perception, control; Python/Zenoh acceptable for high-level coordination and ML inference
