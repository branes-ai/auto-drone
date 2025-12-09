
## AirSim Integration - Complete

## Components Created

  sim_interfaces/airsim_client/
  - AirSimClient.hpp/cpp - High-level wrapper for AirSim RPC API
    - Connect to local or remote AirSim
    - Capture RGB/depth images
    - Get odometry (pose + velocity)
    - Send velocity commands
    - Takeoff/land/hover control
  - AirSimBridge.cpp - Zenoh bridge executable
    - Publishes camera and odometry to Zenoh
    - Subscribes to velocity commands from Zenoh
  - CMakeLists.txt - Conditional build (requires AIRSIM_ROOT)

## Documentation
  - docs/airsim-setup.md - Cloud deployment guide with architecture diagrams
    - AWS/Azure/GCP instance recommendations
    - Headless rendering with Vulkan
    - SSH tunnel for security
    - Bandwidth considerations

## Scripts
  - scripts/install-airsim-linux.sh - Builds AirSim library

## Usage

 ### Build with AirSim (optional)
  export AIRSIM_ROOT=~/airsim-lib/AirSim
  cmake --preset linux-release
  cmake --build --preset linux-release

 ### Run with remote AirSim
  ssh -L 41451:localhost:41451 ubuntu@cloud-ip  # SSH tunnel
  ./build/linux-release/sim_interfaces/airsim_client/airsim_bridge --auto-takeoff

 ### Autonomy stack works unchanged
  ./build/linux-release/autonomy_stack/object_tracking/perception_node

## Key Design

  - AirSim is optional - build works without it
  - Uses same Zenoh topics as mock publishers
  - Autonomy code doesn't need to change between mock and real simulator



## Completed Tasks

  1. Fixed vulkan-utils Package Error

  - Updated scripts/install-airsim-linux.sh to use vulkan-tools (the package was renamed in Ubuntu 22.04+)

  2. Updated for Project AirSim (IAMAI)

  - Install Script (scripts/install-airsim-linux.sh):
    - Changed repo from Microsoft/AirSim to iamaisim/ProjectAirSim
    - Updated build commands for Project AirSim structure (client/cpp/)
    - Uses PROJECTAIRSIM_ROOT instead of AIRSIM_ROOT
  - CMakeLists.txt (sim_interfaces/airsim_client/CMakeLists.txt):
    - Uses PROJECTAIRSIM_ROOT (with AIRSIM_ROOT fallback for compatibility)
    - Updated paths for Project AirSim's client/cpp/ structure
    - Added PROJECTAIRSIM_AVAILABLE compile definition
  - Client Code (from previous session):
    - AirSimClient.hpp/cpp already updated for Project AirSim API:
        - microsoft::projectairsim::client namespace
      - Separate Client, World, Drone classes
      - Arm()/Disarm() methods (not armDisarm(bool))
      - Radians for angles
  - Documentation (docs/airsim-setup.md):
    - Updated for Project AirSim with UE5
    - Added comparison table (old vs new AirSim)
    - Updated all setup instructions

  3. Session Log Updated

  - docs/sessions/2025-12-08-airsim-integration.md reflects the Project AirSim changes

## To Build with Project AirSim

  # Install Project AirSim
  ./scripts/install-airsim-linux.sh ~/projectairsim-lib

  # Build auto-drone with Project AirSim support
  cmake -DPROJECTAIRSIM_ROOT=~/projectairsim-lib/ProjectAirSim --preset linux-release
  cmake --build --preset linux-release

