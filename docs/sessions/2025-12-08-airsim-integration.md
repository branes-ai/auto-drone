# Session: Project AirSim Integration

**Date:** 2025-12-08
**Focus:** Project AirSim (IAMAI) simulator client, cloud deployment documentation

## Goals

1. Create AirSim client library for simulator integration
2. Document cloud GPU instance setup for remote simulation
3. Build Zenoh bridge for AirSim data streaming
4. Update for Project AirSim (not old Microsoft AirSim)

## Key Decisions

### Project AirSim vs Microsoft AirSim
- **Important:** Using Project AirSim (https://github.com/iamaisim/ProjectAirSim)
- The old Microsoft AirSim is deprecated (2022) and no longer maintained
- Project AirSim uses Unreal Engine 5 (vs UE4)
- Different API structure with Client/World/Drone hierarchy

### API Differences
| Feature | Old Microsoft AirSim | Project AirSim (IAMAI) |
|---------|---------------------|------------------------|
| Status | Deprecated | Actively maintained |
| Unreal Engine | UE4 | UE5 |
| Repository | Microsoft/AirSim | iamaisim/ProjectAirSim |
| Namespace | `msr::airlib` | `microsoft::projectairsim::client` |
| Client | Single `MultirotorClient` | Separate `Client`, `World`, `Drone` |
| Arm/Disarm | `armDisarm(bool)` | `Arm()` / `Disarm()` |
| Angles | Degrees | **Radians** |

### Conditional Compilation
- `#ifdef PROJECTAIRSIM_AVAILABLE` guards for client code
- CMake adds definition when Project AirSim headers found
- Stub implementations return false/empty when library unavailable

### Cloud Deployment
- Target: AWS g4dn, Azure NC, GCP N1+T4 instances
- Headless rendering with Vulkan
- SSH tunnel recommended for security
- UE5 + Project AirSim plugin required

## Implementation

### sim_interfaces/airsim_client/
| File | Description |
|------|-------------|
| `AirSimClient.hpp` | High-level wrapper for Project AirSim API |
| `AirSimClient.cpp` | Implementation with conditional compilation |
| `AirSimBridge.cpp` | Zenoh publisher/subscriber for AirSim |
| `CMakeLists.txt` | Conditional build with PROJECTAIRSIM_ROOT |

### scripts/
| File | Description |
|------|-------------|
| `install-airsim-linux.sh` | Clone and build Project AirSim |

### docs/
| File | Description |
|------|-------------|
| `airsim-setup.md` | Cloud deployment guide with architecture diagrams |

## Code Highlights

### Project AirSim Forward Declarations
```cpp
namespace microsoft { namespace projectairsim { namespace client {
    class Client;
    class World;
    class Drone;
}}}

class AirSimClient {
private:
    std::shared_ptr<microsoft::projectairsim::client::Client> client_;
    std::shared_ptr<microsoft::projectairsim::client::World> world_;
    std::shared_ptr<microsoft::projectairsim::client::Drone> drone_;
};
```

### Conditional Compilation Pattern
```cpp
bool AirSimClient::arm() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        // Project AirSim uses separate Arm() method (not armDisarm)
        drone_->Arm();
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}
```

### CMake Configuration
```cmake
# Prefer PROJECTAIRSIM_ROOT, fall back to AIRSIM_ROOT for compatibility
if(EXISTS "${PROJECTAIRSIM_INCLUDE_DIR}")
    target_include_directories(airsim_client SYSTEM PUBLIC
        ${PROJECTAIRSIM_INCLUDE_DIR}
    )
    target_compile_definitions(airsim_client PRIVATE PROJECTAIRSIM_AVAILABLE)
endif()
```

## Issues Encountered

### Package Name Change
- **Issue:** `vulkan-utils` package not found
- **Cause:** Package renamed to `vulkan-tools` in Ubuntu 22.04+
- **Solution:** Updated install script to use `vulkan-tools`

### Wrong AirSim Version
- **Issue:** Initially wrote code for old Microsoft AirSim
- **Cause:** Multiple AirSim forks exist
- **Solution:** Rewrote for Project AirSim API based on client/cpp headers

## Usage

### Build with Project AirSim
```bash
# Install Project AirSim library
./scripts/install-airsim-linux.sh ~/projectairsim-lib

# Build with Project AirSim support
export PROJECTAIRSIM_ROOT=~/projectairsim-lib/ProjectAirSim
cmake --preset linux-release
cmake --build --preset linux-release
```

### Run with Remote Project AirSim
```bash
# On cloud instance: Start UE5 project with Project AirSim plugin
./YourProject.sh -RenderOffScreen

# On local machine: SSH tunnel
ssh -L 41451:localhost:41451 ubuntu@cloud-ip

# Run bridge
./build/linux-release/sim_interfaces/airsim_client/airsim_bridge \
    --host localhost \
    --auto-takeoff
```

### Run Autonomy Stack
```bash
# Same as with mock publisher - Zenoh topics are identical
./build/linux-release/autonomy_stack/object_tracking/perception_node
./build/linux-release/demos/03_object_tracking/target_tracker_node
```

## Zenoh Topics (AirSim Bridge)

| Topic | Direction | Rate | Description |
|-------|-----------|------|-------------|
| `robot/drone/sensor/camera/rgb` | Publish | 30 Hz | RGB camera image |
| `robot/drone/sensor/camera/depth` | Publish | 30 Hz | Depth image (optional) |
| `robot/drone/sensor/state/odom` | Publish | 100 Hz | Pose + velocity |
| `robot/drone/cmd/velocity` | Subscribe | 50 Hz | Velocity commands |

## Cloud Instance Recommendations

| Provider | Instance | GPU | Cost |
|----------|----------|-----|------|
| AWS | g4dn.xlarge | T4 16GB | $0.52/hr |
| Azure | NC4as_T4_v3 | T4 16GB | $0.53/hr |
| GCP | n1-standard-4 + T4 | T4 16GB | $0.55/hr |

**Note:** Project AirSim with UE5 has higher storage requirements (~150GB+).

## Bandwidth Considerations

| Data | Resolution | Rate | Bandwidth |
|------|------------|------|-----------|
| RGB | 640x480 | 30 Hz | ~27 MB/s |
| Depth | 640x480 | 30 Hz | ~18 MB/s |
| Odom | - | 100 Hz | ~3 KB/s |

For low bandwidth connections:
- Reduce resolution (320x240)
- Reduce frame rate (15 Hz)
- Add JPEG compression

## Next Steps

- Deploy test Project AirSim instance on cloud
- Test with running Project AirSim simulator
- Add IMU sensor streaming
- Phase 4: Obstacle avoidance integration
