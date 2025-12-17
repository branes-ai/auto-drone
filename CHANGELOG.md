# Changelog

All notable changes to the auto-drone project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Phase 4: Obstacle Avoidance**
  - `libs/data_types/ProximityData` - 6-direction proximity sensor data type
    - Front/back/left/right/up/down distances (32 bytes serialized)
    - Helper methods: `min_distance()`, `min_horizontal_distance()`, `has_obstacle()`
  - `libs/control_algorithms/ReactiveAvoidance` - Repulsive potential field controller
    - CLEAR/AVOIDING/CRITICAL states based on obstacle proximity
    - Configurable safety distance, critical distance, max avoidance speed
    - Velocity magnitude clamping with direction preservation
  - `autonomy_stack/obstacle_avoidance/CommandArbiter` - Priority-based command arbitration
    - Selects highest priority non-stale command from multiple sources
    - Staleness filtering with configurable timeout (default 500ms)
    - Falls back to hover when no valid commands
  - `demos/04_obstacle_avoidance/` - Obstacle avoidance demo executables
    - `mock_proximity_publisher` - Synthetic proximity data generator
    - `obstacle_avoidance_node` - Reactive avoidance with HIGH priority commands
    - `command_arbiter_node` - Arbitrates nominal vs reactive velocity commands
  - `VelocityCommand::COMMAND_ARBITER` - New source enum value for arbiter output
  - AirSim pillar corridor scene and mission
    - `scene_pillar_corridor.jsonc` - Scene configuration for obstacle course
    - `pillar_corridor_mission.py` - Python mission script with waypoints and proximity simulation

### Changed
- WaypointManager now publishes to `cmd/nominal_vel` topic (was `cmd/velocity`)
  - Enables command arbitration with obstacle avoidance

- **Waypoint Demo Integration with AirSim Bridge**
  - Added `--connect <endpoint>` option to `waypoint_manager` and `waypoint_publisher`
  - Enables remote Zenoh connections (e.g., `--connect tcp/192.168.1.10:7447`)
  - Successfully tested full waypoint following loop through AirSim bridge

- **Drone Commander Tool**
  - `sim_interfaces/airsim_zenoh_bridge/drone_commander.py`
  - Interactive keyboard control (WASD for movement, Q/E for yaw, R/F for altitude)
  - Predefined missions: square, circle, updown
  - Single command mode for scripted operations

- **Mock Bridge for Local Testing**
  - `sim_interfaces/airsim_zenoh_bridge/mock_bridge.py`
  - Simulates AirSim sensor data without actual simulator
  - Useful for testing Zenoh connectivity and serialization on Linux

- **High-Speed Drone Configurations**
  - `robot_quadrotor_highspeed_10ms.jsonc` - 10 m/s (36 km/h)
  - `robot_quadrotor_highspeed_20ms.jsonc` - 20 m/s (72 km/h)
  - `robot_quadrotor_highspeed_30ms.jsonc` - 30 m/s (109 km/h)
  - Corresponding scene files for each configuration
  - Tuned physics: reduced drag coefficient, increased thrust/RPM

### Fixed
- **AirSim Bridge Odometry Parsing**
  - Project AirSim pose data is a dict, not an object with attributes
  - Now correctly parses `pose_data.get('position', {}).get('x', 0)`

- **AirSim Bridge Depth Image Parsing**
  - Depth images use uint16 format (2 bytes/pixel), not float32
  - Added format detection based on data size

- **Circle Mission in Drone Commander**
  - World-frame velocity doesn't curve with yaw rate
  - Fixed to compute tangential velocity from position relative to circle center

- **Windows CI Zenoh Test Segfaults**
  - Tests were crashing with SIGSEGV during Zenoh session initialization
  - Root cause: New `SessionConfig` code was setting `mode = "peer"` by default
  - The `zc_config_insert_json5(Z_CONFIG_MODE_KEY)` call caused issues on Windows CI
  - Solution: Changed default `SessionConfig::mode` to empty string (use Zenoh defaults)
  - Default sessions now skip mode configuration, matching previous working behavior

- **Python Zenoh Bridge for Project AirSim**
  - `sim_interfaces/airsim_zenoh_bridge/` - Python bridge for remote AirSim connectivity
    - `airsim_zenoh_bridge.py` - Main bridge connecting Project AirSim to Zenoh network
    - `data_types.py` - Binary serialization matching C++ formats (ImageData, Odometry, VelocityCommand)
    - `test_bridge.py` - Python test client for verifying bridge connectivity
    - `test_bridge_cpp.cpp` - C++ test client for Linux-side verification
  - Enables cross-machine communication: Windows (AirSim) ↔ Linux (Autonomy Stack)
  - Workaround for Project AirSim's localhost-only Python API limitation

- **Remote Zenoh Endpoint Configuration**
  - `SessionConfig` struct in `libs/zenoh_interface/ZenohInterface.hpp`
    - `connect_endpoints` - Remote endpoints to connect to (e.g., "tcp/192.168.1.10:7447")
    - `listen_endpoints` - Local endpoints to listen on
    - Static helpers: `SessionConfig::local()`, `SessionConfig::connect_to(endpoint)`
  - Backward compatible - default constructor uses local scouting

- **AirSim Integration**
  - `sim_interfaces/airsim_client` - AirSim RPC client wrapper
    - `AirSimClient` class for camera capture, odometry, velocity commands
    - Support for local and remote AirSim instances
    - `airsim_bridge` executable for Zenoh-AirSim bridging
  - `docs/airsim-setup.md` - Cloud deployment guide
    - AWS/Azure/GCP instance recommendations
    - Headless rendering configuration
    - Network and bandwidth considerations
  - `scripts/install-airsim-linux.sh` - AirSim build helper

- **Phase 3: Object Tracking**
  - `libs/data_types` - ObjectDetection data type
    - 2D bounding box and 3D world coordinates
    - ObjectDetectionList for frame results with serialization
  - `autonomy_stack/object_tracking` - Perception library
    - `PerceptionEngine` for 2D-to-3D projection
    - `ColorBlobDetector` for HSV color-based blob detection
    - `perception_node` executable for processing camera images
  - `libs/control_algorithms` - TargetTracker controller
    - Visual servoing for image-based target tracking
    - Distance maintenance with 3D tracking support
  - `demos/03_object_tracking` - Object tracking demo
    - `mock_target_publisher` - Generates images with moving red target
    - `target_tracker_node` - Tracks detected objects with velocity commands

- **Phase 2: Basic Autonomy**
  - `libs/control_algorithms` - PID control library
    - `PIDController` with anti-windup, derivative filtering, feedforward support
    - `PositionController` for 3D position control (X, Y, Z axes)
    - `YawController` with angle wrapping for ±π discontinuity
    - Comprehensive unit tests
  - `libs/data_types` - New waypoint and velocity data types
    - `Waypoint` struct with position, yaw, speed, and behavior flags
    - `WaypointList` for waypoint sequences with serialization
    - `VelocityCommand` with priority levels and source tracking
  - `demos/02_waypoint_following` - Waypoint following demo
    - `simulated_drone` - Velocity integration, odometry publishing at 100Hz
    - `waypoint_manager` - PID-based waypoint controller
    - `waypoint_publisher` - Square/circle pattern generator for testing

## [0.1.1] - 2025-12-08

### Added
- **Phase 1: Communication Core**
  - GitHub Actions CI workflow for Linux, macOS, and Windows
  - Multi-camera webcam publisher with Skydio-style camera positioning
  - Cross-platform argument parsing (replaced POSIX getopt)

### Fixed
- Windows CI: Added OpenCV and Zenoh bin directories to PATH for DLL resolution
- GitHub Actions: Corrected `dtolnay/rust-toolchain@stable` action name
- Linux camera detection: Filter metadata nodes, verify frame capture capability
- USB bandwidth: Documentation for multi-camera USB controller separation

### Changed
- Renamed "Odom" labels to "Odometry" throughout codebase

## [0.1.0] - 2025-12-01

### Added
- Initial project skeleton with CMake presets for cross-platform builds
- `libs/zenoh_interface` - RAII wrapper for Zenoh session, pub/sub
- `libs/data_types` - ImageData and Odometry with serialization
- `demos/01_sensor_streaming` - Mock publisher, viewer nodes (GUI and headless)
- Architecture documentation for future phases:
  - Object tracking
  - Obstacle avoidance
  - Visual SLAM
  - Visual waypoint navigation
  - Swarm coordination

### Infrastructure
- CMake 3.16+ with C++20 support
- Catch2 v3.5.0 for unit testing
- Zenoh-C for pub/sub communication
- OpenCV 4.x for image processing

[Unreleased]: https://github.com/branes-ai/auto-drone/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/branes-ai/auto-drone/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/branes-ai/auto-drone/releases/tag/v0.1.0
