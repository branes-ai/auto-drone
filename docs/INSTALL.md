# Installation Guide

This guide covers installing the required dependencies for the auto-drone project on Linux, macOS, and Windows.

## Prerequisites

All platforms require:
- **CMake 3.16+**
- **C++20 compatible compiler**
- **Ninja** build system (recommended)
- **Git**

## Quick Start

Use the provided install scripts in `scripts/`:

```bash
# Linux (Ubuntu/Debian)
./scripts/install-deps-linux.sh

# macOS
./scripts/install-deps-macos.sh

# Windows (PowerShell as Administrator)
.\scripts\install-deps-windows.ps1
```

---

## Linux (Ubuntu/Debian)

### System Packages

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    pkg-config \
    libopencv-dev \
    curl
```

### Rust Toolchain (required for Zenoh-C)

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

### Zenoh-C Library

```bash
# Clone and build Zenoh-C
git clone https://github.com/eclipse-zenoh/zenoh-c.git
cd zenoh-c
mkdir build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
sudo ninja install
sudo ldconfig
```

### Verify Installation

```bash
pkg-config --modversion zenohc
pkg-config --modversion opencv4
```

---

## Linux (Fedora/RHEL)

### System Packages

```bash
sudo dnf install -y \
    gcc-c++ \
    cmake \
    ninja-build \
    git \
    pkg-config \
    opencv-devel \
    curl
```

### Rust and Zenoh-C

Follow the same Rust and Zenoh-C installation steps as Ubuntu above.

---

## macOS

### Homebrew Packages

```bash
# Install Homebrew if not present
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake ninja opencv pkg-config
```

### Rust Toolchain

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

### Zenoh-C Library

```bash
git clone https://github.com/eclipse-zenoh/zenoh-c.git
cd zenoh-c
mkdir build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
sudo ninja install
```

### Verify Installation

```bash
pkg-config --modversion zenohc
pkg-config --modversion opencv4
```

---

## Windows

### Option 1: vcpkg (Recommended)

vcpkg simplifies dependency management on Windows.

```powershell
# Install vcpkg
git clone https://github.com/Microsoft/vcpkg.git C:\vcpkg
cd C:\vcpkg
.\bootstrap-vcpkg.bat

# Install OpenCV
.\vcpkg install opencv4:x64-windows

# Integrate with system
.\vcpkg integrate install
```

### Option 2: Manual Installation

#### Visual Studio

Install Visual Studio 2022 with "Desktop development with C++" workload.

#### CMake and Ninja

```powershell
# Using winget
winget install Kitware.CMake
winget install Ninja-build.Ninja

# Or download from:
# CMake: https://cmake.org/download/
# Ninja: https://ninja-build.org/
```

#### OpenCV

1. Download prebuilt binaries from https://opencv.org/releases/
2. Extract to `C:\opencv`
3. Add `C:\opencv\build\x64\vc16\bin` to PATH
4. Set environment variable: `OpenCV_DIR=C:\opencv\build`

#### Rust Toolchain

Download and run: https://win.rustup.rs/

```powershell
# Verify installation
rustc --version
cargo --version
```

#### Zenoh-C Library

```powershell
git clone https://github.com/eclipse-zenoh/zenoh-c.git
cd zenoh-c
mkdir build
cd build

# Using Visual Studio generator
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build . --config Release

# Install (run as Administrator)
cmake --install . --config Release
```

Alternatively, with Ninja:

```powershell
# Open "x64 Native Tools Command Prompt for VS 2022"
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
ninja install
```

### Environment Variables (Windows)

Add to system PATH:
- `C:\Program Files\zenohc\bin`
- `C:\opencv\build\x64\vc16\bin`

Set CMAKE_PREFIX_PATH:
```powershell
[Environment]::SetEnvironmentVariable("CMAKE_PREFIX_PATH", "C:\Program Files\zenohc;C:\opencv\build", "User")
```

---

## Building the Project

Once dependencies are installed, use CMake presets:

```bash
# Linux/macOS with Ninja
cmake --preset linux-release    # or macos-release
cmake --build --preset linux-release

# Windows with Visual Studio
cmake --preset windows-release
cmake --build --preset windows-release

# Windows with Ninja (from VS Developer Command Prompt)
cmake --preset windows-ninja-release
cmake --build --preset windows-ninja-release
```

Or manually:

```bash
cmake -B build -S . -GNinja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## Troubleshooting

### Zenoh-C not found

```bash
# Linux: Ensure ldconfig was run
sudo ldconfig

# Check pkg-config path
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# Or specify CMAKE_PREFIX_PATH
cmake -B build -DCMAKE_PREFIX_PATH=/usr/local ...
```

### OpenCV not found (Windows)

```powershell
# Set OpenCV_DIR environment variable
$env:OpenCV_DIR = "C:\opencv\build"
cmake -B build -DOpenCV_DIR="C:\opencv\build" ...
```

### Rust/Cargo not in PATH

```bash
# Linux/macOS
source ~/.cargo/env

# Windows - restart terminal or:
$env:Path += ";$env:USERPROFILE\.cargo\bin"
```

### Permission denied during install

```bash
# Linux/macOS - use sudo for system-wide install
sudo ninja install

# Or install to user directory
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local
ninja install
export CMAKE_PREFIX_PATH=$HOME/.local:$CMAKE_PREFIX_PATH
```
