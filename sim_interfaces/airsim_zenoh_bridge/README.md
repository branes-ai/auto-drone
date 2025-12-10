# Project AirSim Zenoh Bridge

Python bridge that connects Project AirSim to a Zenoh network, enabling remote autonomy stacks to receive sensor data and send control commands.

## Architecture

```
[Windows Workstation]                    [Linux Workstation]
┌─────────────────────────────┐          ┌─────────────────────────────┐
│  Project AirSim (Simulator) │          │  Autonomy Stack             │
│           ↕                 │          │  (SLAM, Planning, Control)  │
│  airsim_zenoh_bridge.py     │          │           ↕                 │
│           ↕                 │          │  Zenoh Subscribers/         │
│  Zenoh (listen 0.0.0.0:7447)│◄────────►│  Publishers                 │
└─────────────────────────────┘   TCP    └─────────────────────────────┘
```

## Requirements

- Python 3.9 (Project AirSim requirement)
- Project AirSim Python client (`projectairsim`)
- Zenoh Python (`eclipse-zenoh`)
- NumPy, OpenCV

## Installation

On the Windows machine running Project AirSim:

```bash
# Activate your Project AirSim Python environment
cd path\to\auto-drone\sim_interfaces\airsim_zenoh_bridge

# Install dependencies
pip install -r requirements.txt
```

Note: The `projectairsim` package should already be installed from the Project AirSim setup.

## Usage

### Basic Usage (Windows side)

Run on the same machine as Project AirSim:

```bash
# Listen for Zenoh connections from remote machines
python airsim_zenoh_bridge.py --zenoh-listen tcp/0.0.0.0:7447
```

### With Auto-Takeoff

```bash
python airsim_zenoh_bridge.py --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff --altitude 5.0
```

### Custom Rates

```bash
python airsim_zenoh_bridge.py \
    --zenoh-listen tcp/0.0.0.0:7447 \
    --rgb-rate 15 \
    --depth-rate 15 \
    --odom-rate 50
```

### Connecting from Linux (Autonomy Stack)

Your C++ autonomy code should configure Zenoh to connect to the Windows bridge:

```cpp
// In your Zenoh configuration
z_config_insert(config, Z_CONFIG_CONNECT_KEY, "tcp/192.168.1.10:7447");
```

Or using zenoh CLI tools for testing:

```bash
# Subscribe to odometry
z_sub -e tcp/192.168.1.10:7447 -k "robot/drone/sensor/state/odom"

# Subscribe to images
z_sub -e tcp/192.168.1.10:7447 -k "robot/drone/sensor/camera/rgb"
```

## Command Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--zenoh-connect` | None | Zenoh endpoint to connect to |
| `--zenoh-listen` | None | Zenoh endpoint to listen on |
| `--robot-id` | `drone` | Robot ID for topic namespacing |
| `--camera` | `front_center` | AirSim camera name |
| `--rgb-rate` | 30 | RGB image publish rate (Hz) |
| `--depth-rate` | 30 | Depth image publish rate (Hz) |
| `--odom-rate` | 100 | Odometry publish rate (Hz) |
| `--width` | 640 | Image width |
| `--height` | 480 | Image height |
| `--auto-takeoff` | False | Auto takeoff on start |
| `--altitude` | 10.0 | Takeoff altitude (m) |

## Zenoh Topics

### Published (Sensor Data)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `robot/{id}/sensor/camera/rgb` | ImageData | 30 Hz | RGB camera image |
| `robot/{id}/sensor/camera/depth` | ImageData | 30 Hz | Depth image (normalized) |
| `robot/{id}/sensor/state/odom` | Odometry | 100 Hz | Pose and orientation |

### Subscribed (Commands)

| Topic | Type | Description |
|-------|------|-------------|
| `robot/{id}/cmd/velocity` | VelocityCommand | Body-frame velocity command |

## Data Formats

All data is serialized in little-endian binary format matching the C++ data types.

### ImageData
```
[width:u32][height:u32][channels:u32][encoding:u32][pixels:bytes]
```

### Odometry
```
[x:f32][y:f32][z:f32][roll:f32][pitch:f32][yaw:f32][timestamp_us:u64]
Total: 32 bytes
```

### VelocityCommand
```
[vx:f32][vy:f32][vz:f32][yaw_rate:f32][priority:u8][source:u8][pad:2][timestamp_us:u64]
Total: 28 bytes
```

## Firewall Configuration

Open port 7447 on Windows for Zenoh:

```powershell
# PowerShell (Administrator)
New-NetFirewallRule -DisplayName "Zenoh Bridge" -Direction Inbound -Protocol TCP -LocalPort 7447 -Action Allow

PS C:\WINDOWS\system32> New-NetFirewallRule -DisplayName "Zenoh Bridge" -Direction Inbound -Protocol TCP -LocalPort 7447 -Action Allow

Name                          : {7274769c-5c42-489f-8d44-769ae2f6737f}
DisplayName                   : Zenoh Bridge
Description                   :
DisplayGroup                  :
Group                         :
Enabled                       : True
Profile                       : Any
Platform                      : {}
Direction                     : Inbound
Action                        : Allow
EdgeTraversalPolicy           : Block
LooseSourceMapping            : False
LocalOnlyMapping              : False
Owner                         :
PrimaryStatus                 : OK
Status                        : The rule was parsed successfully from the store. (65536)
EnforcementStatus             : NotApplicable
PolicyStoreSource             : PersistentStore
PolicyStoreSourceType         : Local
RemoteDynamicKeywordAddresses : {}
PolicyAppId                   :
PackageFamilyName             :
```

## Troubleshooting

### Connection Issues

1. Verify Project AirSim is running
2. Check Windows firewall allows port 7447
3. Test connectivity: `nc -zv <windows-ip> 7447`

### No Sensor Data

1. Check bridge statistics output
2. Verify camera name matches AirSim configuration
3. Ensure drone is armed and has taken off

### Command Timeout

Commands timeout after 0.5s if no new command received. The drone will hover automatically.


## ● Test Results Summary

  All local tests passed:

  | Test                                | Result | Details                                                                      |
  |-------------------------------------|--------|------------------------------------------------------------------------------|
  | C++ test client (test_bridge_cpp)   | ✓ Pass | 300 RGB frames (30 Hz), 299 depth frames (30 Hz), 991 odom messages (~99 Hz) |
  | Python test client (test_bridge.py) | ✓ Pass oper serializati| Same rates, pron/deserialization                             |
  | Command transmission                | ✓ Pass | Velocity commands sent and received by mock bridge                           |
  | Data serialization                  | ✓ Pass | Binary format matches between Python and C++                                 |

  What was tested:
  1. Zenoh TCP connectivity (tcp/127.0.0.1:7447)
  2. Topic publishing: robot/drone/sensor/camera/rgb, depth, state/odom
  3. Topic subscribing: robot/drone/cmd/velocierialization of ty
  4. Binary sImageData, Odometry, VelocityCommand

  Created file:
  - sim_inzenoh_bridge/mocterfaces/airsim_k_bridge.py - simulates AirSim bridge for local testing

  ---

## Next Steps for Real AirSim Testing

  To test with the real Project AirSim on Windows:

  1. On Windows machine (running Project AirSim):

```ps
  # Open firewall port
  New-NetFirewallRule -DisplayName "Zenoh Bridge" -Direction Inbound -Protocol TCP -LocalPort 7447 -Action Allow

  # Run the bridge
  python airsim_zenoh_bridge.py --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff
```

  2. On Linux machine (autonomy stack):
```bash
  # Test connectivity
  ./build/linux-release/sim_interfaces/airsim_zenoh_bridge/test_bridge_cpp \
      --connect tcp/<WINDOWS_IP>:7447 --duration 10 --send-commands
```

  3. Verify the test output shows:
    - RGB/Depth frames expected rates
    - Odometry at ~100 Hz
    - Drone responds to velocity commands

