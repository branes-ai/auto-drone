# Session: Project AirSim Zenoh Bridge

**Date:** 2025-12-09
**Focus:** Remote connectivity between Project AirSim (Windows) and Autonomy Stack (Linux)

## Goals

1. Establish network connectivity to Project AirSim on remote Windows workstation
2. Create Python-based Zenoh bridge for cross-machine communication
3. Update C++ Zenoh interface to support remote endpoints
4. Build test utilities to verify bridge connectivity

## Problem Statement

Project AirSim (IAMAI) currently only supports Python clients and localhost connections. Our autonomy stack runs on a Linux workstation, while Project AirSim runs on a Windows workstation with GPU. We needed a bridge solution to enable cross-machine sensor data streaming and command forwarding.

## Architecture

```
[Windows Workstation A]                    [Linux Workstation B]
┌─────────────────────────────┐            ┌─────────────────────────────┐
│  Project AirSim (Simulator) │            │  Autonomy Stack             │
│           ↕                 │            │  (SLAM, Planning, Control)  │
│  airsim_zenoh_bridge.py     │            │           ↕                 │
│           ↕                 │            │  Zenoh Subscribers/         │
│  Zenoh (listen 0.0.0.0:7447)│◄──────────►│  Publishers                 │
└─────────────────────────────┘    TCP     └─────────────────────────────┘
```

## Key Discoveries

### Project AirSim Network Configuration

- **Port:** 50052 (not 41451 like old Microsoft AirSim)
- **Settings:** No `settings.json` file - configuration is programmatic
- **Limitation:** Python API only supports localhost connections
- **Workaround:** Python Zenoh bridge running on same machine as simulator

### Root Cause Analysis Process

1. Initial connection timeout from Linux to Windows
2. Checked Windows firewall - port 41451 was open but wrong port
3. Used `netstat` to find actual listening port: 50052
4. Discovered 50052 works, 50051 times out (firewall)
5. Confirmed Project AirSim doesn't use `settings.json` (that's old AirSim)
6. Solution: Python bridge with Zenoh for network transport

## Implementation

### New Files: sim_interfaces/airsim_zenoh_bridge/

| File | Description |
|------|-------------|
| `airsim_zenoh_bridge.py` | Main bridge - connects AirSim to Zenoh network |
| `data_types.py` | Python serialization matching C++ binary format |
| `test_bridge.py` | Python test client for connectivity verification |
| `test_bridge_cpp.cpp` | C++ test client for Linux-side testing |
| `requirements.txt` | Python dependencies (eclipse-zenoh, numpy, opencv) |
| `CMakeLists.txt` | Build configuration for C++ test |
| `README.md` | Usage documentation |

### Modified Files

| File | Changes |
|------|---------|
| `libs/zenoh_interface/ZenohInterface.hpp` | Added `SessionConfig` struct |
| `libs/zenoh_interface/ZenohInterface.cpp` | Implemented remote endpoint configuration |
| `CMakeLists.txt` | Added airsim_zenoh_bridge subdirectory |

## Code Highlights

### SessionConfig for Remote Endpoints

```cpp
struct SessionConfig {
    std::vector<std::string> connect_endpoints;  // e.g., "tcp/192.168.1.10:7447"
    std::vector<std::string> listen_endpoints;   // e.g., "tcp/0.0.0.0:7447"
    std::string mode = "peer";

    static SessionConfig local() { return SessionConfig{}; }
    static SessionConfig connect_to(const std::string& endpoint) {
        SessionConfig cfg;
        cfg.connect_endpoints.push_back(endpoint);
        return cfg;
    }
};
```

### Python Data Type Serialization

```python
@dataclass
class Odometry:
    """Matches C++ Odometry binary format (32 bytes)"""
    x: float; y: float; z: float
    roll: float; pitch: float; yaw: float
    timestamp_us: int

    FORMAT = '<ffffffQ'  # 6 floats + 1 uint64, little-endian

    def serialize(self) -> bytes:
        return struct.pack(self.FORMAT, self.x, self.y, self.z,
                          self.roll, self.pitch, self.yaw, self.timestamp_us)
```

### Bridge Main Loop

```python
# Sensor publishing threads
threading.Thread(target=self._sensor_loop, args=(self._publish_rgb, 30.0))
threading.Thread(target=self._sensor_loop, args=(self._publish_depth, 30.0))
threading.Thread(target=self._sensor_loop, args=(self._publish_odometry, 100.0))

# Command application thread (50 Hz with timeout safety)
threading.Thread(target=self._command_loop)
```

## Windows Firewall Commands

```powershell
# Open Zenoh bridge port
New-NetFirewallRule -DisplayName "Zenoh Bridge" -Direction Inbound -Protocol TCP -LocalPort 7447 -Action Allow

# Open Project AirSim API port (if needed)
New-NetFirewallRule -DisplayName "Project AirSim API" -Direction Inbound -Protocol TCP -LocalPort 50052 -Action Allow
```

## Testing Commands

### On Windows (with Project AirSim)

```bash
cd sim_interfaces/airsim_zenoh_bridge
pip install -r requirements.txt
python airsim_zenoh_bridge.py --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff
```

### On Linux (Autonomy Stack)

```bash
# Python test
python test_bridge.py --connect tcp/192.168.1.10:7447

# C++ test
./build/linux-release/sim_interfaces/airsim_zenoh_bridge/test_bridge_cpp \
    --connect tcp/192.168.1.10:7447

# With commands (moves the drone)
python test_bridge.py --connect tcp/192.168.1.10:7447 --send-commands
```

## Zenoh Topics

| Topic | Direction | Rate | Format |
|-------|-----------|------|--------|
| `robot/drone/sensor/camera/rgb` | Bridge → Linux | 30 Hz | ImageData (16-byte header + pixels) |
| `robot/drone/sensor/camera/depth` | Bridge → Linux | 30 Hz | ImageData (normalized uint8) |
| `robot/drone/sensor/state/odom` | Bridge → Linux | 100 Hz | Odometry (32 bytes) |
| `robot/drone/cmd/velocity` | Linux → Bridge | 50 Hz | VelocityCommand (28 bytes) |

## Binary Format Compatibility

All data types use little-endian serialization matching C++ `struct.pack`:

| Type | C++ | Python | Size |
|------|-----|--------|------|
| ImageData | `<IIII` + data | `struct.pack('<IIII', w, h, ch, enc)` | 16 + w*h*ch |
| Odometry | 6 floats + u64 | `struct.pack('<ffffffQ', ...)` | 32 bytes |
| VelocityCommand | 4 floats + 2 u8 + pad + u64 | `struct.pack('<ffffBBxxQ', ...)` | 28 bytes |

## Issues Encountered

### Wrong Port Assumption
- **Issue:** Assumed port 41451 from old Microsoft AirSim documentation
- **Solution:** Used `netstat -an | findstr LISTENING` to find actual port 50052

### Missing settings.json
- **Issue:** Looked for `settings.json` to configure remote binding
- **Cause:** Project AirSim uses programmatic configuration, not file-based
- **Solution:** Python bridge handles network transport via Zenoh

### Firewall Complexity
- **Issue:** Port 50052 worked but 50051 timed out
- **Solution:** Only 50052 needed for our use case; opened 7447 for Zenoh

## Next Steps

1. Deploy and test with actual Project AirSim instance running
2. Measure latency and bandwidth in real-world conditions
3. Add JPEG compression option for bandwidth-constrained networks
4. Integrate with existing autonomy stack demos
5. Add IMU sensor streaming to bridge
