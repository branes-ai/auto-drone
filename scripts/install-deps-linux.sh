#!/bin/bash
# install-deps-linux.sh
# Installs dependencies for auto-drone on Linux (Ubuntu/Debian)

set -e

echo "=== Auto-Drone Dependency Installer (Linux) ==="
echo ""

# Detect distribution
if [ -f /etc/os-release ]; then
    . /etc/os-release
    DISTRO=$ID
else
    echo "Warning: Cannot detect distribution, assuming Debian-based"
    DISTRO="debian"
fi

echo "Detected distribution: $DISTRO"
echo ""

# Install system packages
install_system_packages() {
    echo ">>> Installing system packages..."

    case $DISTRO in
        ubuntu|debian|linuxmint|pop)
            sudo apt update
            sudo apt install -y \
                build-essential \
                cmake \
                ninja-build \
                git \
                pkg-config \
                libopencv-dev \
                curl \
                clang
            ;;
        fedora|rhel|centos|rocky|alma)
            sudo dnf install -y \
                gcc-c++ \
                cmake \
                ninja-build \
                git \
                pkg-config \
                opencv-devel \
                curl \
                clang
            ;;
        arch|manjaro)
            sudo pacman -Syu --noconfirm \
                base-devel \
                cmake \
                ninja \
                git \
                pkg-config \
                opencv \
                curl \
                clang
            ;;
        opensuse*)
            sudo zypper install -y \
                gcc-c++ \
                cmake \
                ninja \
                git \
                pkg-config \
                opencv-devel \
                curl \
                clang
            ;;
        *)
            echo "Error: Unsupported distribution: $DISTRO"
            echo "Please install manually: cmake, ninja, git, opencv, pkg-config"
            exit 1
            ;;
    esac

    echo ">>> System packages installed."
}

# Install Rust toolchain
install_rust() {
    if command -v rustc &> /dev/null; then
        echo ">>> Rust already installed: $(rustc --version)"
        echo ">>> Updating Rust..."
        rustup update
    else
        echo ">>> Installing Rust toolchain..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
        source "$HOME/.cargo/env"
    fi

    echo ">>> Rust installed: $(rustc --version)"
}

# Build and install Zenoh-C
install_zenohc() {
    local ZENOH_VERSION="${ZENOH_VERSION:-main}"
    local INSTALL_PREFIX="${ZENOH_INSTALL_PREFIX:-/usr/local}"
    local BUILD_DIR="/tmp/zenoh-c-build"

    echo ">>> Installing Zenoh-C (version: $ZENOH_VERSION)..."

    # Check if already installed
    if pkg-config --exists zenohc 2>/dev/null; then
        local installed_version=$(pkg-config --modversion zenohc)
        echo ">>> Zenoh-C already installed: $installed_version"
        read -p ">>> Reinstall? [y/N] " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            return 0
        fi
    fi

    # Clean up any previous build
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"

    # Clone Zenoh-C
    echo ">>> Cloning Zenoh-C repository..."
    git clone --depth 1 --branch "$ZENOH_VERSION" \
        https://github.com/eclipse-zenoh/zenoh-c.git "$BUILD_DIR/zenoh-c" || \
    git clone --depth 1 https://github.com/eclipse-zenoh/zenoh-c.git "$BUILD_DIR/zenoh-c"

    cd "$BUILD_DIR/zenoh-c"

    # Build
    echo ">>> Building Zenoh-C..."
    mkdir build && cd build
    cmake .. -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"
    ninja

    # Install
    echo ">>> Installing Zenoh-C to $INSTALL_PREFIX..."
    sudo ninja install
    sudo ldconfig

    # Cleanup
    cd /
    rm -rf "$BUILD_DIR"

    echo ">>> Zenoh-C installed: $(pkg-config --modversion zenohc)"
}

# Verify installation
verify_installation() {
    echo ""
    echo "=== Verifying Installation ==="

    local success=true

    # Check CMake
    if command -v cmake &> /dev/null; then
        echo "[OK] CMake: $(cmake --version | head -1)"
    else
        echo "[FAIL] CMake not found"
        success=false
    fi

    # Check Ninja
    if command -v ninja &> /dev/null; then
        echo "[OK] Ninja: $(ninja --version)"
    else
        echo "[FAIL] Ninja not found"
        success=false
    fi

    # Check C++ compiler
    if command -v g++ &> /dev/null; then
        echo "[OK] g++: $(g++ --version | head -1)"
    elif command -v clang++ &> /dev/null; then
        echo "[OK] clang++: $(clang++ --version | head -1)"
    else
        echo "[FAIL] No C++ compiler found"
        success=false
    fi

    # Check Rust
    if command -v rustc &> /dev/null; then
        echo "[OK] Rust: $(rustc --version)"
    else
        echo "[FAIL] Rust not found"
        success=false
    fi

    # Check OpenCV
    if pkg-config --exists opencv4 2>/dev/null; then
        echo "[OK] OpenCV: $(pkg-config --modversion opencv4)"
    elif pkg-config --exists opencv 2>/dev/null; then
        echo "[OK] OpenCV: $(pkg-config --modversion opencv)"
    else
        echo "[FAIL] OpenCV not found"
        success=false
    fi

    # Check Zenoh-C
    if pkg-config --exists zenohc 2>/dev/null; then
        echo "[OK] Zenoh-C: $(pkg-config --modversion zenohc)"
    else
        echo "[FAIL] Zenoh-C not found"
        success=false
    fi

    echo ""
    if $success; then
        echo "=== All dependencies installed successfully! ==="
        echo ""
        echo "You can now build the project:"
        echo "  cmake --preset linux-release"
        echo "  cmake --build --preset linux-release"
    else
        echo "=== Some dependencies are missing. Please check the errors above. ==="
        exit 1
    fi
}

# Main
main() {
    install_system_packages
    install_rust

    # Source cargo environment
    if [ -f "$HOME/.cargo/env" ]; then
        source "$HOME/.cargo/env"
    fi

    install_zenohc
    verify_installation
}

main "$@"
