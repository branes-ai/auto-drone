# Changelog

All notable changes to the auto-drone project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
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

## [0.1.1] - 2024-12-08

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

## [0.1.0] - 2024-12-01

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
