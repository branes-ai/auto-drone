# Project AirSim Cloud Setup Guide

This guide describes how to set up **Project AirSim (IAMAI)** on a cloud GPU instance and connect to it from your local auto-drone development environment.

> **NOTE:** This is for [Project AirSim](https://github.com/iamaisim/ProjectAirSim), the actively maintained fork that uses Unreal Engine 5. This is NOT the old Microsoft AirSim which is no longer maintained.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         REMOTE/CLOUD GPU INSTANCE                           │
│     GPU Workstation or Cloud Instance (AWS g4dn / Azure NC / GCP N1+T4)     │
│                                                                             │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │              Unreal Engine 5 + Project AirSim Plugin                  │  │
│  │                                                                       │  │
│  │   ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐   │  │
│  │   │   Drone     │    │  Cameras    │    │   Physics Simulation    │   │  │
│  │   │   Model     │    │  RGB/Depth  │    │   IMU, GPS, Barometer   │   │  │
│  │   └─────────────┘    └─────────────┘    └─────────────────────────┘   │  │
│  │                                                                       │  │
│  │                    RPC Server (Port 41451)                            │  │
│  └───────────────────────────────┬───────────────────────────────────────┘  │
│                                  │                                          │
│                                  │ TCP/IP                                   │
└──────────────────────────────────┼──────────────────────────────────────────┘
                                   │
                            ┌──────┴──────┐
                            │   Network   │
                            │  (Internet) │
                            └──────┬──────┘
                                   │
┌──────────────────────────────────┼──────────────────────────────────────────┐
│                                  │                                          │
│                        LOCAL DEVELOPMENT MACHINE                            │
│                                  |                                          │
│  ┌───────────────────────────────┴───────────────────────────────────────┐  │
│  │                        airsim_client                                  │  │
│  │                                                                       │  │
│  │   ┌─────────────────┐              ┌─────────────────────────────┐    │  │
│  │   │  Project AirSim │              │   Zenoh Publisher           │    │  │
│  │   │  RPC Client     │─────────────►│   - camera/rgb              │    │  │
│  │   │  (TCP:41451)    │              │   - sensor/state/odom       │    │  │
│  │   └─────────────────┘              └──────────────┬──────────────┘    │  │
│  │                                                   │                   │  │
│  │   ┌─────────────────┐              ┌──────────────┴──────────────┐    │  │
│  │   │  Velocity Cmd   │◄─────────────│   Zenoh Subscriber          │    │  │
│  │   │  Executor       │              │   - cmd/velocity            │    │  │
│  │   └─────────────────┘              └─────────────────────────────┘    │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                  │                                          │
│                                  │ Zenoh (local)                            │
│                                  ▼                                          │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                         Autonomy Stack                                │  │
│  │                                                                       │  │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │  │
│  │   │  viewer     │  │  perception │  │  waypoint   │  │  target     │  │  │
│  │   │  _node      │  │  _node      │  │  _manager   │  │  _tracker   │  │  │
│  │   └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘  │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Zenoh Data Flow

```
                    Cloud Project AirSim              Local Machine
                    ────────────────────              ─────────────

                    ┌──────────────┐
                    │Project AirSim│
                    │  Simulator   │
                    │    (UE5)     │
                    └──────┬───────┘
                           │ RPC (41451)
                           ▼
                    ┌──────────┐
                    │ airsim   │──────► robot/drone/sensor/camera/rgb
                    │ _client  │──────► robot/drone/sensor/camera/depth
                    │          │──────► robot/drone/sensor/state/odom
                    │          │──────► robot/drone/sensor/imu
                    │          │◄────── robot/drone/cmd/velocity
                    └──────────┘
                         │
                         │ Zenoh (local pub/sub)
                         ▼
              ┌──────────┴──────────┐
              ▼                     ▼
        ┌──────────┐          ┌──────────┐
        │ viewer   │          │perception│
        │ _node    │          │ _node    │
        └──────────┘          └────┬─────┘
                                   │
                                   ▼
                              ┌──────────┐
                              │ target   │
                              │ _tracker │
                              └────┬─────┘
                                   │
                                   ▼
                         robot/drone/cmd/velocity
                                   │
                                   ▼
                            (back to airsim_client)
```

## Project AirSim vs Old AirSim

| Feature | Old Microsoft AirSim | Project AirSim (IAMAI) |
|---------|---------------------|------------------------|
| Status | Deprecated (2022) | Actively maintained |
| Unreal Engine | UE4 | UE5 |
| Repository | Microsoft/AirSim | iamaisim/ProjectAirSim |
| API | Single client class | Client/World/Drone hierarchy |
| Arm/Disarm | `armDisarm(bool)` | Separate `Arm()` / `Disarm()` |
| Angles | Degrees | Radians |
| Default Port | 41451 | 41451 |

## Cloud Instance Requirements

### Minimum Specifications
| Resource | Minimum | Recommended |
|----------|---------|-------------|
| GPU | NVIDIA T4 (16GB) | NVIDIA A10G (24GB) |
| vCPUs | 4 | 8+ |
| RAM | 16 GB | 32 GB |
| Storage | 150 GB SSD | 250 GB SSD |
| Network | 1 Gbps | 10 Gbps |

> **Note:** Project AirSim with UE5 has higher storage requirements than the old AirSim.

### Recommended Cloud Instances

| Provider | Instance Type | GPU | vCPU | RAM | Cost (approx) |
|----------|---------------|-----|------|-----|---------------|
| AWS | g4dn.xlarge | T4 | 4 | 16 GB | $0.52/hr |
| AWS | g4dn.2xlarge | T4 | 8 | 32 GB | $0.75/hr |
| Azure | NC4as_T4_v3 | T4 | 4 | 28 GB | $0.53/hr |
| GCP | n1-standard-4 + T4 | T4 | 4 | 15 GB | $0.55/hr |

## Setup Instructions

### 1. Launch Cloud Instance

#### AWS (Ubuntu 22.04)
```bash
# Launch g4dn.xlarge with Ubuntu 22.04 AMI
# Ensure security group allows:
#   - Port 22 (SSH)
#   - Port 41451 (Project AirSim RPC) - restrict to your IP
#   - Port 5900 (VNC, optional)
```

#### Install NVIDIA Drivers
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install NVIDIA drivers
sudo apt install -y nvidia-driver-535

# Reboot
sudo reboot

# Verify
nvidia-smi
```

### 2. Install Project AirSim Dependencies

```bash
# Install Vulkan for headless rendering
# NOTE: vulkan-utils was renamed to vulkan-tools in Ubuntu 22.04+
sudo apt install -y vulkan-tools libvulkan1 vulkan-validationlayers mesa-vulkan-drivers

# Install other dependencies
sudo apt install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    xorg-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libboost-all-dev \
    libeigen3-dev

# For VNC (optional, for visual debugging)
sudo apt install -y tigervnc-standalone-server tigervnc-common
```

### 3. Install Unreal Engine 5

Project AirSim requires Unreal Engine 5. You have two options:

#### Option A: Epic Games Launcher (Recommended)
1. Register for an Epic Games account
2. Link your GitHub account to access UE5 source
3. Follow [Epic's UE5 Linux installation guide](https://docs.unrealengine.com/5.0/en-US/linux-development-requirements-for-unreal-engine/)

#### Option B: Build from Source
```bash
# Clone UE5 (requires linked GitHub account)
git clone https://github.com/EpicGames/UnrealEngine.git
cd UnrealEngine
git checkout 5.3  # or latest stable

# Setup and build
./Setup.sh
./GenerateProjectFiles.sh
make
```

### 4. Install Project AirSim

```bash
# Clone Project AirSim
git clone https://github.com/iamaisim/ProjectAirSim.git
cd ProjectAirSim

# Build the C++ client library
cd client/cpp
mkdir build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja

# Build the UE5 plugin (from Project AirSim root)
cd ../../..
# Follow Project AirSim documentation for UE5 plugin build
```

### 5. Configure Project AirSim

Create `~/Documents/ProjectAirSim/settings.json`:

```json
{
  "SeeDocsAt": "https://github.com/iamaisim/ProjectAirSim",
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "ViewMode": "NoDisplay",
  "ClockSpeed": 1.0,

  "Vehicles": {
    "Drone": {
      "VehicleType": "SimpleFlight",
      "AutoCreate": true,
      "EnableCollisionPassthrough": false,
      "EnableCollisions": true,

      "Cameras": {
        "front_center": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 640,
              "Height": 480,
              "FOV_Degrees": 70
            },
            {
              "ImageType": 2,
              "Width": 640,
              "Height": 480,
              "FOV_Degrees": 70
            }
          ],
          "X": 0.25, "Y": 0, "Z": -0.1,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      },

      "Sensors": {
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "Gps": {
          "SensorType": 3,
          "Enabled": true
        },
        "Barometer": {
          "SensorType": 1,
          "Enabled": true
        }
      }
    }
  },

  "ApiServerPort": 41451
}
```

### 6. Run Project AirSim (Headless)

```bash
# Set display for headless rendering
export DISPLAY=:0

# Start virtual display (if no physical display)
Xvfb :0 -screen 0 1920x1080x24 &

# Run your UE5 project with Project AirSim plugin
# The exact command depends on your UE5 project setup
./YourProject.sh -RenderOffScreen -WINDOWED -ResX=640 -ResY=480

# Or with explicit Vulkan
./YourProject.sh -RenderOffScreen -vulkan
```

### 7. Verify Project AirSim is Running

```bash
# Check if RPC server is listening
netstat -tlnp | grep 41451

# Test with Python client
pip install projectairsim  # or build from source
python -c "
from projectairsim import client
c = client.Client()
c.connect()
print('Connected to Project AirSim!')
"
```

### 8. Network Configuration

#### SSH Tunnel (Recommended for Security)
```bash
# On local machine, create SSH tunnel
ssh -L 41451:localhost:41451 ubuntu@your-cloud-instance-ip

# Now connect to localhost:41451 from airsim_client
```

#### Direct Connection (Less Secure)
```bash
# On cloud instance, open firewall
sudo ufw allow from YOUR_LOCAL_IP to any port 41451

# Connect directly to cloud IP from airsim_client
./airsim_publisher --host your-cloud-instance-ip
```

## Bandwidth Considerations

| Data Type | Resolution | Rate | Bandwidth |
|-----------|------------|------|-----------|
| RGB Image | 640x480 | 30 Hz | ~27 MB/s |
| Depth Image | 640x480 | 30 Hz | ~18 MB/s |
| Odometry | - | 100 Hz | ~3 KB/s |
| IMU | - | 200 Hz | ~5 KB/s |

**Total:** ~45 MB/s (360 Mbps) for full sensor suite

### Reducing Bandwidth
1. Lower resolution (320x240 = ~7 MB/s)
2. Lower frame rate (15 Hz = ~14 MB/s)
3. JPEG compression (10:1 = ~3 MB/s)
4. Request images on-demand vs streaming

## Latency Expectations

| Connection Type | Latency |
|-----------------|---------|
| Same region (e.g., us-east) | 20-50 ms |
| Cross-region | 50-100 ms |
| Cross-continent | 100-200 ms |

For real-time control, latency < 100ms is recommended.

## Troubleshooting

### Project AirSim Won't Start
```bash
# Check GPU
nvidia-smi

# Check Vulkan
vulkaninfo | head -20

# Check logs
cat ~/Documents/ProjectAirSim/Logs/*.log
```

### Can't Connect from Local Machine
```bash
# Check if Project AirSim is listening
sudo netstat -tlnp | grep 41451

# Check firewall
sudo ufw status

# Test connectivity
nc -zv cloud-instance-ip 41451
```

### High Latency / Dropped Frames
- Use JPEG compression for images
- Reduce image resolution
- Reduce capture rate
- Use closer cloud region

## Cost Optimization

1. **Use Spot Instances:** 70-90% cost savings
2. **Auto-shutdown:** Stop instance when not in use
3. **Smaller Instance:** g4dn.xlarge is often sufficient
4. **Preemptible (GCP):** Similar to spot instances

```bash
# Example: Start instance only when needed
aws ec2 start-instances --instance-ids i-1234567890abcdef0
# ... do your work ...
aws ec2 stop-instances --instance-ids i-1234567890abcdef0
```

## Resources

- [Project AirSim GitHub](https://github.com/iamaisim/ProjectAirSim)
- [Project AirSim Documentation](https://github.com/iamaisim/ProjectAirSim/wiki)
- [Unreal Engine 5 Documentation](https://docs.unrealengine.com/5.0/)
