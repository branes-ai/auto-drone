# Session: Phase 1 - CI and Multi-Camera Support

**Date:** 2025-12-08
**Focus:** GitHub Actions CI, multi-camera webcam publisher, cross-platform fixes

## Goals

1. Create cross-platform GitHub Actions CI workflow
2. Add multi-camera webcam publisher (Skydio-style: front, back, left, right, top, bottom)
3. Fix Windows build issues
4. Rename "Odom" to "Odometry" throughout codebase

## Key Decisions

### CI Workflow
- Target platforms: Linux (ubuntu-24.04), macOS (macos-14), Windows (windows-2022)
- Cache Zenoh-C builds to speed up CI runs
- Use CMake presets for consistent builds across platforms

### Multi-Camera Support
- Probe cameras by attempting frame capture (filters Linux metadata nodes)
- Use V4L2 backend explicitly on Linux for better compatibility
- Support configurable camera indices via `--cameras` flag
- Wildcard subscription pattern: `robot/drone/sensor/camera/*`

### Cross-Platform Compatibility
- Replace POSIX `getopt.h` with simple argument parsing loop
- Add OpenCV and Zenoh bin directories to Windows PATH for DLL resolution

## Implementation

### Files Created
- `.github/workflows/ci.yml` - Cross-platform CI workflow

### Files Modified
- `demos/01_sensor_streaming/WebcamPublisher.cpp` - Multi-camera support
- `demos/01_sensor_streaming/ViewerNode.cpp` - Wildcard subscriptions, grid display
- `demos/01_sensor_streaming/ViewerNodeHeadless.cpp` - Multi-camera support
- `demos/01_sensor_streaming/MockPublisher.cpp` - Odometry rename

## Testing

- CI workflow validated on all three platforms
- Multi-camera tested with two USB cameras on Linux
- Verified camera detection skips metadata nodes (e.g., /dev/video1)

## Issues Encountered

### GitHub Actions
- **Issue:** `dtolnay/rust-action` not found
- **Solution:** Correct action is `dtolnay/rust-toolchain@stable`

### Windows Build
- **Issue:** `getopt.h` not available on Windows
- **Solution:** Replaced with cross-platform for-loop argument parsing

- **Issue:** Exit code 0xc0000135 (DLL not found)
- **Solution:** Added OpenCV and Zenoh bin directories to `GITHUB_PATH`

### Linux Camera Detection
- **Issue:** Only 1 camera detected when 2 present
- **Cause:** /dev/video1 was metadata node, not capture device
- **Solution:** Verify frame capture in `probe_cameras()`, skip non-capture devices

### USB Bandwidth
- **Issue:** Second camera detected but 0 frames captured
- **Cause:** Both cameras on same USB 2.0 hub
- **Solution:** User moved camera to different USB controller

## Zenoh Topics

| Topic | Description |
|-------|-------------|
| `robot/drone/sensor/camera/*` | Camera images (wildcard for multi-camera) |
| `robot/drone/sensor/state/odom` | Odometry data (renamed from `odom`) |

## Next Steps

- Proceed to Phase 2: Basic Autonomy (PID, waypoints)
