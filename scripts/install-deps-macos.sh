#!/bin/bash
# install-deps-macos.sh
# Installs dependencies for auto-drone on macOS

set -e

echo "=== Auto-Drone Dependency Installer (macOS) ==="
echo ""

# Check for Homebrew
check_homebrew() {
    if ! command -v brew &> /dev/null; then
        echo ">>> Homebrew not found. Installing..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

        # Add to PATH for Apple Silicon
        if [[ $(uname -m) == "arm64" ]]; then
            echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
            eval "$(/opt/homebrew/bin/brew shellenv)"
        fi
    fi
    echo ">>> Homebrew: $(brew --version | head -1)"
}

# Install system packages via Homebrew
install_system_packages() {
    echo ">>> Installing system packages via Homebrew..."

    brew update

    # Install required packages
    brew install cmake ninja opencv pkg-config

    # Optional: Install clang from LLVM for newer C++ features
    # brew install llvm

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

    # Cleanup
    cd /
    rm -rf "$BUILD_DIR"

    echo ">>> Zenoh-C installed."
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
    if command -v clang++ &> /dev/null; then
        echo "[OK] clang++: $(clang++ --version | head -1)"
    else
        echo "[FAIL] clang++ not found"
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
        echo "  cmake --preset macos-release"
        echo "  cmake --build --preset macos-release"
    else
        echo "=== Some dependencies are missing. Please check the errors above. ==="
        exit 1
    fi
}

# Main
main() {
    # Check we're on macOS
    if [[ "$(uname)" != "Darwin" ]]; then
        echo "Error: This script is for macOS only."
        exit 1
    fi

    check_homebrew
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
