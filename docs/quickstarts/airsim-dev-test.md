# Project AirSim Quick Start

We have the following configuration:
|  Machine      |  IP Address  |            Purpose              |
|---------------|--------------|---------------------------------|
| Workstation A | 192.168.1.10 | Run Project AirSim simulator    |
| Workstation B | 192.168.1.21 | Run your auto-drone client code |

The AirSim integration leverages a Zenoh bridge architecture. The recommended testing approach is as follows:

```verbatim
  [Workstation A]                    [Workstation B]

  Project AirSim  <--RPC-->  AirSimBridge  <--Zenoh-->  Your Autonomy Code
     (Simulator)              (on A or B)                 (Subscribers/Publishers)
```


  Files Created

  sim_interfaces/airsim_zenoh_bridge/
  ├── airsim_zenoh_bridge.py   # Main bridge script
  ├── data_types.py            # Serialization matching C++ formats
  ├── requirements.txt         # Python dependencies
  └── README.md                # Usage documentation

  How to Use

  On Windows (Workstation A - with Project AirSim)

  1. Copy the airsim_zenoh_bridge folder to Windows
  2. Install dependencies in your Python 3.9 environment:
  pip install eclipse-zenoh numpy opencv-python
  3. Open firewall port for Zenoh:
  New-NetFirewallRule -DisplayName "Zenoh Bridge" -Direction Inbound -Protocol TCP -LocalPort 7447 -Action Allow
  4. Run the bridge:
  python airsim_zenoh_bridge.py --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff

  On Linux (Workstation B - this machine)

  Your existing C++ autonomy code needs to connect to the Windows Zenoh bridge. Test with:

  # Verify connectivity
  nc -zv 192.168.1.10 7447

  # Test with zenoh tools (if installed)
  z_sub -e tcp/192.168.1.10:7447 -k "robot/drone/sensor/state/odom"

  Key Features

  - Sensor publishing: RGB (30Hz), Depth (30Hz), Odometry (100Hz)
  - Command receiving: Velocity commands with timeout/hover safety
  - Binary compatible: Uses same serialization format as your C++ data types
  - Configurable rates: Adjust for bandwidth/latency tradeoffs


## Testing Strategy

Files Created
```verbatim
  sim_interfaces/airsim_zenoh_bridge/ (Python bridge package):
  ├── __init__.py              # Package init
  ├── airsim_zenoh_bridge.py   # Main bridge (runs on Windows)
  ├── data_types.py            # Binary serialization matching C++
  ├── test_bridge.py           # Python test client (runs on Linux)
  ├── test_bridge_cpp.cpp      # C++ test client (runs on Linux)
  ├── requirements.txt         # Python dependencies
  ├── CMakeLists.txt           # Build for C++ test
  └── README.md                # Documentation
```

Modified Files

  - libs/zenoh_interface/ZenohInterface.hpp - Added SessionConfig struct for remote endpoints
  - libs/zenoh_interface/ZenohInterface.cpp - Implemented configurable connect/listen endpoints
  - CMakeLists.txt - Added airsim_zenoh_bridge subdirectory

  ---
How to Test

**Step 1**: On Windows (Workstation A)

  1. Copy sim_interfaces/airsim_zenoh_bridge/ folder to Windows
  2. Install dependencies:
  pip install eclipse-zenoh numpy opencv-python
  3. Open firewall port:
  New-NetFirewallRule -DisplayName "Zenoh Bridge" -Direction Inbound -Protocol TCP -LocalPort 7447 -Action Allow
  4. Start Project AirSim
  5. Run the bridge:
  python airsim_zenoh_bridge.py --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff

**Step 2**: On Linux (Workstation B)

  Option A: Python test (no build required):
  cd sim_interfaces/airsim_zenoh_bridge
  pip install eclipse-zenoh numpy
  python test_bridge.py --connect tcp/192.168.1.10:7447

  Option B: C++ test (uses your existing build):
  ./build/linux-release/sim_interfaces/airsim_zenoh_bridge/test_bridge_cpp \
      --connect tcp/192.168.1.10:7447

  With commands (to actually move the drone):
  python test_bridge.py --connect tcp/192.168.1.10:7447 --send-commands

Expected Output

  ============================================================
  Bridge Test Statistics (elapsed: 10.0s)
  ============================================================
  RGB Frames:      300  ( 30.0 Hz)  Size: (640, 480)
  Depth Frames:    300  ( 30.0 Hz)  Size: (640, 480)
  Odom Messages:  1000  (100.0 Hz)

  Last Odometry:
    Position: x=   0.000, y=   0.000, z=  -5.000
    Rotation: roll= 0.000, pitch= 0.000, yaw= 0.000

  ✓ Bridge connection: OK





-------------------------------------------------------------------------------------------
==============================================================================================

**Step 1**: Determine Where to Run the Bridge

We have two options:

  | Option | AirSimBridge Location             | Pros                          | Cons                          |
  |--------|-----------------------------------|-------------------------------|-------------------------------|
  | A      | On Workstation A (with simulator) | Lower latency for sensor data | Need to build code on A       |
  | B      | On Workstation B (drone facsimile)        | Easier development iteration  | RPC over network adds latency |


**Step 2**: Configure Network Connection



**Step 3**: Test Basic Connectivity

  First, verify you can reach Workstation A from B:

```bash

(p311) stillwater@sw-21:~/dev/branes/clones/auto-drone$ ping 192.168.1.10
PING 192.168.1.10 (192.168.1.10) 56(84) bytes of data.
64 bytes from 192.168.1.10: icmp_seq=1 ttl=128 time=0.497 ms
64 bytes from 192.168.1.10: icmp_seq=2 ttl=128 time=0.745 ms
64 bytes from 192.168.1.10: icmp_seq=3 ttl=128 time=0.653 ms

```

# Open AirSim port

Here's how to open port 41451 on Windows for Project AirSim:

##  Method 1: Windows Defender Firewall (GUI)

  1. Press Win + R, type wf.msc, press Enter
  2. Click Inbound Rules in the left panel
  3. Click New Rule... in the right panel
  4. Select Port → Next
  5. Select TCP, enter 41451 → Next
  6. Select Allow the connection → Next
  7. Check all profiles (Domain, Private, Public) → Next
  8. Name it: Project AirSim API → Finish

## Method 2: PowerShell (Faster)

Run PowerShell as Administrator:

```ps
> New-NetFirewallRule -DisplayName "Project AirSim API" -Direction Inbound -Protocol TCP -LocalPort 41451 -Action Allow

PS C:\WINDOWS\system32> New-NetFirewallRule -DisplayName "Project AirSim API" -Direction Inbound -Protocol TCP -LocalPort 41451 -Action Allow


Name                          : {63e102b9-5a2c-455c-95cd-c513abe4b3d0}
DisplayName                   : Project AirSim API
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
## Method 3: Command Prompt (Admin)

```cmd
> netsh advfirewall firewall add rule name="Project AirSim API" dir=in action=allow protocol=tcp localport=41451
```

## Verify the Rule

  Get-NetFirewallRule -DisplayName "Project AirSim API"

## Test from Linux Workstation

  After opening the port, test connectivity from your Linux box:

  nc -zv <windows-ip> 41451

  If successful, you'll see "Connection succeeded" or similar.

---
Note: If you're on a managed network (corporate/university), there may also be network-level firewalls blocking traffic between machines. In that case, you'd need to contact your IT team or ensure both machines are on the same subnet/VLAN.

# Check if AirSim port is open

```bash
  nc -zv 192.168.1.10 41451
```

**Step 4**: Build and Run the AirSimBridge

  On your current workstation (B):

  cd /home/stillwater/dev/branes/clones/auto-drone

  # Build (if not already built)
  cmake --preset linux-release
  cmake --build --preset linux-release

  # Run the bridge pointing to remote AirSim
  ./build/linux-release/sim_interfaces/airsim_client/AirSimBridge \
    --host 192.168.x.x \
    --port 41451 \
    --auto-takeoff \
    --altitude 5.0

**Step 5**: Monitor Zenoh Topics

  In a separate terminal, you can use the existing demo viewers or write a quick test subscriber:

  # Use the headless viewer from demo 01
  ./build/linux-release/demos/01_sensor_streaming/viewer_node_headless

  # Or check if zenoh tools are available
  z_sub -k "robot/drone/**"

**Step 6**: Send Test Commands

You can publish velocity commands to test control:

# If you have zenoh CLI tools
  z_pub -k "robot/drone/cmd/velocity" -v "<serialized command>"
