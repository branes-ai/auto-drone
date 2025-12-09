#!/bin/bash
# install-airsim-linux.sh
# Clones and builds Project AirSim (IAMAI) for use with auto-drone
#
# NOTE: This is for Project AirSim (https://github.com/iamaisim/ProjectAirSim),
#       NOT the old Microsoft AirSim which is no longer maintained.
#
# Usage:
#   ./scripts/install-airsim-linux.sh [install_dir]
#
# Example:
#   ./scripts/install-airsim-linux.sh ~/projectairsim-lib
#
# After installation, build auto-drone with:
#   cmake -DPROJECTAIRSIM_ROOT=~/projectairsim-lib/ProjectAirSim --preset linux-release

set -e

INSTALL_DIR="${1:-$HOME/projectairsim-lib}"
PROJECTAIRSIM_BRANCH="main"

echo "=== Project AirSim Installation Script ==="
echo "Install directory: $INSTALL_DIR"
echo "Branch: $PROJECTAIRSIM_BRANCH"
echo ""
echo "NOTE: Project AirSim requires Unreal Engine 5.x"
echo "      See: https://github.com/iamaisim/ProjectAirSim"
echo ""

# Create install directory
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"

# Install dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    libboost-all-dev \
    libeigen3-dev \
    wget \
    unzip \
    python3 \
    python3-pip

# Install Vulkan for headless rendering
# NOTE: vulkan-utils was renamed to vulkan-tools in newer Ubuntu versions
echo "Installing Vulkan dependencies..."
sudo apt-get install -y \
    vulkan-tools \
    libvulkan1 \
    vulkan-validationlayers \
    mesa-vulkan-drivers

# Clone Project AirSim
if [ -d "ProjectAirSim" ]; then
    echo "ProjectAirSim directory exists, pulling latest..."
    cd ProjectAirSim
    git fetch
    git checkout $PROJECTAIRSIM_BRANCH
    git pull origin $PROJECTAIRSIM_BRANCH
    cd ..
else
    echo "Cloning Project AirSim..."
    git clone https://github.com/iamaisim/ProjectAirSim.git
    cd ProjectAirSim
    git checkout $PROJECTAIRSIM_BRANCH
    cd ..
fi

# Build Project AirSim C++ client library
cd ProjectAirSim

echo "Building Project AirSim C++ client library..."

# Project AirSim uses CMake for the client library
# The client is in client/cpp/
if [ -d "client/cpp" ]; then
    cd client/cpp
    mkdir -p build
    cd build
    cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
    ninja
    cd ../../..
else
    echo "WARNING: client/cpp directory not found"
    echo "Project AirSim structure may have changed"
    echo "Please check: https://github.com/iamaisim/ProjectAirSim"
fi

# Also build the shared library if available
if [ -f "CMakeLists.txt" ]; then
    echo "Building Project AirSim shared components..."
    mkdir -p build
    cd build
    cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
    ninja
    cd ..
fi

echo ""
echo "=== Project AirSim Installation Complete ==="
echo ""
echo "Project AirSim installed to: $INSTALL_DIR/ProjectAirSim"
echo ""
echo "To build auto-drone with Project AirSim support:"
echo "  export PROJECTAIRSIM_ROOT=$INSTALL_DIR/ProjectAirSim"
echo "  cmake --preset linux-release"
echo "  cmake --build --preset linux-release"
echo ""
echo "Or specify PROJECTAIRSIM_ROOT directly:"
echo "  cmake -DPROJECTAIRSIM_ROOT=$INSTALL_DIR/ProjectAirSim --preset linux-release"
echo ""
echo "IMPORTANT: Project AirSim uses Unreal Engine 5."
echo "To run simulations, you'll need to:"
echo "  1. Install Unreal Engine 5 from Epic Games Launcher"
echo "  2. Build the Project AirSim UE5 plugin"
echo "  3. Run a UE5 project with the plugin enabled"
echo ""
echo "See: https://github.com/iamaisim/ProjectAirSim for full documentation"
echo ""
