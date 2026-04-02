# Virtual World Prerequisites

Running autonomous drone missions in a virtual world requires NVIDIA Isaac Sim, which in turn requires an NVIDIA RTX GPU. This guide walks you through discovering what hardware you have, understanding the deployment options, and setting up the right topology for your situation.

## Step 1: Discover Your Hardware

Before choosing a deployment topology, you need to know what GPU resources are available on each machine in your development setup. Run these commands on every machine you plan to use.

### Linux

```bash
# Check for any NVIDIA GPU
nvidia-smi
```

If `nvidia-smi` is found, you'll see output like:

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 550.54.14    Driver Version: 550.54.14    CUDA Version: 12.4     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA RTX 4090     Off  | 00000000:01:00.0  On |                  Off |
|  0%   38C    P8    18W / 450W |    512MiB / 24576MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
```

If you see `command not found`, check whether there is GPU hardware without drivers:

```bash
# Check for NVIDIA hardware on the PCI bus (doesn't require drivers)
lspci | grep -i nvidia

# Check what GPU hardware exists (any vendor)
lspci | grep -i -E 'vga|3d|display'
```

**Interpreting the results:**

| Result | Meaning |
|--------|---------|
| `nvidia-smi` shows an RTX card | GPU ready — install Isaac Sim on this machine |
| `lspci` shows NVIDIA but `nvidia-smi` fails | GPU present but drivers not installed |
| `lspci` shows only Intel/AMD integrated | No discrete GPU — cannot run Isaac Sim locally |
| Empty `lspci` output for NVIDIA | No NVIDIA hardware at all |

### Windows

Open PowerShell and run:

```powershell
# Quick check
nvidia-smi

# Or via DirectX diagnostics
dxdiag
```

In Task Manager, go to the **Performance** tab and look for a **GPU** entry. It will show the GPU model and dedicated memory.

### macOS

macOS does not support NVIDIA GPUs (since 2019). Isaac Sim cannot run natively on macOS. Use a cloud GPU instance or a separate Linux/Windows machine with an NVIDIA GPU.

## Step 2: Understand the Deployment Topology

The auto-drone platform separates **simulation** (GPU-intensive) from **autonomy** (CPU-intensive). They communicate over Zenoh, which works identically whether the components are on the same machine or across a network. This means you can mix and match based on what hardware is available.

### Topology A: Everything on One Machine (Simplest)

**Requires**: A Linux machine with an NVIDIA RTX GPU (minimum RTX 2070, recommended RTX 3080+).

```
┌──────────────────────────────────────────────┐
│  Linux Workstation (with NVIDIA RTX GPU)      │
│                                               │
│  Isaac Sim (headless or GUI)                  │
│       │                                       │
│       │ Zenoh (localhost, shared memory)       │
│       │                                       │
│  Autonomy Stack (C++ / Python)                │
└──────────────────────────────────────────────┘
```

**Pros**: Lowest latency via Zenoh shared memory. Simplest setup.
**Cons**: Needs a powerful machine. No remote access for visualization without extra setup.

### Topology B: Windows GPU + Linux Autonomy (Mirrors AirSim Setup)

**Requires**: A Windows machine with an NVIDIA RTX GPU, and a Linux machine (GPU not required).

This is the same split used by the existing Project AirSim integration, so it will feel familiar.

```
┌──────────────────────┐    Zenoh     ┌───────────────────────┐
│  Linux Server        │◄────────────►│  Windows Workstation   │
│  (no GPU required)   │  (network)   │  (NVIDIA RTX GPU)      │
│                      │              │                        │
│  Autonomy stack      │              │  Isaac Sim (GUI mode)  │
│  C++ libs + Python   │              │  Zenoh bridge          │
│  Mission framework   │              │  Visualization built-in│
└──────────────────────┘              └────────────────────────┘
```

**Pros**: Uses existing hardware. GUI mode gives interactive scene editing. Same pattern as AirSim.
**Cons**: Network latency for sensor data. Windows Isaac Sim has some feature limitations vs. Linux.

### Topology C: Cloud GPU + Linux Autonomy + Browser Visualization

**Requires**: A Linux machine (GPU not required), a cloud GPU VM, and a browser on any machine.

This is the right choice when no local machine has an NVIDIA GPU.

```
┌──────────────────────┐    Zenoh     ┌───────────────────────┐
│  Linux Server        │◄────────────►│  Cloud GPU VM          │
│  (no GPU required)   │  (network)   │  (NVIDIA RTX GPU)      │
│                      │              │                        │
│  Autonomy stack      │              │  Isaac Sim (headless)  │
│  C++ libs + Python   │              │  Docker container      │
│  Mission framework   │              │  WebRTC streaming      │
└──────────────────────┘              └───────────┬───────────┘
                                                   │ WebRTC
                                      ┌────────────┴──────────┐
                                      │  Any Machine (browser) │
                                      │  WebRTC viewer at      │
                                      │  http://<ip>:8211/     │
                                      │  streaming/webrtc-client│
                                      └────────────────────────┘
```

**Pros**: No local GPU needed. Scalable. Reproducible via Docker.
**Cons**: Cloud costs (~$0.50-$1.00/hr). Network latency. Requires NGC API key.

### Topology D: Hybrid (Development + CI/CD)

For teams, combine Topology B for daily development with Topology C for automated testing:

- **Developer machines**: Windows or Linux with GPU, running Isaac Sim locally
- **CI/CD pipeline**: Cloud GPU VMs running Isaac Sim headless in Docker for automated mission validation

## Step 3: Choose Your Cloud Instance (Topology C)

If you need a cloud GPU instance, here are the recommended options sorted by cost.

> **Prices as of 2026-04-02.** Cloud GPU pricing changes frequently. Verify current rates at the provider links below before provisioning.

| Provider | Instance Type | GPU | VRAM | Cost/hr (approx) | Notes |
|----------|--------------|-----|------|-------------------|-------|
| [AWS](https://aws.amazon.com/ec2/pricing/on-demand/) | `g5.xlarge` | 1x A10G | 24 GB | $1.00 | Best balance |
| [AWS](https://aws.amazon.com/ec2/pricing/on-demand/) | `g6.xlarge` | 1x L4 | 24 GB | $0.80 | Newer, slightly cheaper |
| [GCP](https://cloud.google.com/compute/gpus-pricing) | `g2-standard-4` | 1x L4 | 24 GB | $0.70 | Cheapest option |
| [Azure](https://azure.microsoft.com/en-us/pricing/details/virtual-machines/linux/) | `Standard_NC4as_T4_v3` | 1x T4 | 16 GB | $0.53 | Budget option (T4 is minimum spec) |
| [Azure](https://azure.microsoft.com/en-us/pricing/details/virtual-machines/linux/) | `Standard_NCS_v3` | 1x V100 | 16 GB | $3.00 | Overkill for single-drone sim |

**Minimum GPU requirements for Isaac Sim:**
- GPU: NVIDIA RTX 2070 or higher (RTX, Quadro RTX, or datacenter A-series/L-series)
- VRAM: 8 GB minimum, 16 GB+ recommended
- Driver: 525.60+ (Linux), 528.24+ (Windows)

## Step 4: Install Isaac Sim

### Option A: pip install (Linux with GPU, simplest)

Requires Python 3.10 and an NVIDIA RTX GPU on the same machine.

```bash
# Create a dedicated virtual environment (Isaac Sim pins many dependencies)
python3.10 -m venv ~/venv/isaac-sim
source ~/venv/isaac-sim/bin/activate

# Install Isaac Sim (as of 4.5+, pip-installable)
pip install isaacsim
```

### Option B: Docker container (Linux with GPU, recommended for reproducibility)

```bash
# 1. Ensure NVIDIA Container Toolkit is installed
#    See: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# 2. Verify GPU is visible inside Docker
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi

# 3. Log into NGC (requires free API key from https://ngc.nvidia.com/setup/api-key)
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>

# 4. Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:4.5.0

# 5a. Run headless (no display needed)
docker run --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:4.5.0 \
  ./runheadless.native.sh

# 5b. Run headless with WebRTC streaming (for remote visualization)
docker run --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:4.5.0 \
  ./runheadless.webrtc.sh
# Then open http://<host-ip>:8211/streaming/webrtc-client in a browser
```

**Important**: Use `--network=host` — Docker bridge networking breaks WebRTC.

### Option C: Windows installer (Windows with GPU)

1. Install the [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/)
2. In the launcher, go to the **Exchange** tab and install **Isaac Sim**
3. Launch Isaac Sim from the launcher — it opens with a full GUI

### Option D: Omniverse Streaming Client (Windows viewer for remote Isaac Sim)

If Isaac Sim runs headless on a remote Linux machine, install the native streaming client on Windows for lower-latency visualization than WebRTC:

1. Download from the [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/) Exchange tab
2. Connect to `<remote-host-ip>:48010`

## Step 5: Install Zenoh on the Isaac Sim Machine

Isaac Sim and the autonomy stack communicate over Zenoh. Install the Zenoh Python bindings on whichever machine runs Isaac Sim:

```bash
pip install eclipse-zenoh
```

For cross-machine Zenoh communication, either:
- Use Zenoh's automatic peer discovery (works on the same LAN, multicast)
- Or configure explicit endpoints:

```python
import zenoh

# On the Isaac Sim machine (publisher)
config = zenoh.Config()
config.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')
session = zenoh.open(config)

# On the autonomy machine (subscriber)
config = zenoh.Config()
config.insert_json5("connect/endpoints", '["tcp/<isaac-sim-host>:7447"]')
session = zenoh.open(config)
```

## Step 6: Verify the Setup

Run this quick smoke test to confirm Isaac Sim can publish sensor data to Zenoh and your autonomy host can receive it.

**On the Isaac Sim machine:**

```python
# verify_isaac_zenoh.py
import zenoh
import numpy as np
import time

try:
    # This import only works where Isaac Sim is installed
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    print("[OK] Isaac Sim launched headless")
    simulation_app.close()
except ImportError:
    print("[SKIP] Isaac Sim not installed on this machine")

# Test Zenoh independently
session = zenoh.open(zenoh.Config())
pub = session.declare_publisher("robot/drone/sensor/test")
pub.put(b"hello from isaac sim host")
print("[OK] Zenoh publisher working")
time.sleep(1)
session.close()
print("[OK] All checks passed")
```

**On the autonomy machine:**

```python
# verify_zenoh_subscriber.py
import zenoh
import time

received = []

def listener(sample):
    received.append(sample.payload.to_bytes())
    print(f"[OK] Received: {sample.payload.to_bytes().decode()}")

config = zenoh.Config()
# Uncomment and set IP if on a different machine:
# config.insert_json5("connect/endpoints", '["tcp/<isaac-sim-host>:7447"]')

session = zenoh.open(config)
sub = session.declare_subscriber("robot/drone/sensor/test", listener)

print("Waiting for messages (run verify_isaac_zenoh.py on the Isaac Sim host)...")
time.sleep(10)
session.close()

if received:
    print("[OK] Cross-machine Zenoh communication verified")
else:
    print("[FAIL] No messages received — check network and Zenoh endpoints")
```

## Decision Flowchart

```
Does your Linux server have an NVIDIA RTX GPU?
  │
  ├── YES ──► Topology A (all-in-one) or Topology D (hybrid)
  │           Install via pip or Docker on this machine.
  │
  └── NO
       │
       Does your Windows workstation have an NVIDIA RTX GPU?
         │
         ├── YES ──► Topology B (Windows GPU + Linux autonomy)
         │           Install Isaac Sim on Windows via Omniverse Launcher.
         │           Same pattern as your existing Project AirSim setup.
         │
         └── NO ──► Topology C (cloud GPU)
                    Spin up a cloud GPU VM (AWS g5.xlarge recommended).
                    Run Isaac Sim in Docker. View via WebRTC in browser.
```

## Troubleshooting

### `nvidia-smi: command not found` but `lspci` shows NVIDIA hardware

NVIDIA drivers are not installed. On Ubuntu:

```bash
sudo apt install nvidia-driver-550   # or latest available
sudo reboot
```

### Docker: `could not select device driver "nvidia"`

NVIDIA Container Toolkit is not installed or not configured:

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### WebRTC viewer shows black screen

- Ensure `--network=host` was used when launching the container
- Check that port 8211 is not blocked by a firewall
- Try a different browser (Chrome works best)

### Zenoh peers can't find each other across machines

- Verify network connectivity: `ping <other-host>`
- Use explicit endpoints instead of multicast discovery (see Step 5)
- Check that port 7447 is not blocked by a firewall

## Next Steps

Once your environment is verified:

1. Proceed to [Experiment 1](../designs/isaac-sim-integration-architecture.md#experiment-1-headless-multirotor-architecture-c--1-2-weeks) — headless multirotor with Zenoh
2. Review the [Isaac Sim architecture options](../designs/isaac-sim-integration-architecture.md) for the full integration plan
3. Check [issue #3](https://github.com/branes-ai/auto-drone/issues/3) for detailed setup tasks
