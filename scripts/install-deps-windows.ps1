# install-deps-windows.ps1
# Installs dependencies for auto-drone on Windows
# Run as Administrator for system-wide installation

#Requires -Version 5.1

param(
    [switch]$UseVcpkg,
    [string]$VcpkgRoot = "C:\vcpkg",
    [string]$OpenCVPath = "C:\opencv",
    [string]$ZenohInstallPath = "C:\Program Files\zenohc"
)

$ErrorActionPreference = "Stop"

Write-Host "=== Auto-Drone Dependency Installer (Windows) ===" -ForegroundColor Cyan
Write-Host ""

# Check if running as Administrator
function Test-Administrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

# Install Chocolatey package manager
function Install-Chocolatey {
    if (Get-Command choco -ErrorAction SilentlyContinue) {
        Write-Host ">>> Chocolatey already installed" -ForegroundColor Green
        return
    }

    Write-Host ">>> Installing Chocolatey..." -ForegroundColor Yellow
    Set-ExecutionPolicy Bypass -Scope Process -Force
    [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
    Invoke-Expression ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

    # Refresh environment
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")
}

# Install basic tools via Chocolatey
function Install-BasicTools {
    Write-Host ">>> Installing basic tools..." -ForegroundColor Yellow

    # Install CMake
    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
        choco install cmake --installargs 'ADD_CMAKE_TO_PATH=System' -y
    }
    else {
        Write-Host ">>> CMake already installed" -ForegroundColor Green
    }

    # Install Ninja
    if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
        choco install ninja -y
    }
    else {
        Write-Host ">>> Ninja already installed" -ForegroundColor Green
    }

    # Install Git
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
        choco install git -y
    }
    else {
        Write-Host ">>> Git already installed" -ForegroundColor Green
    }

    # Refresh environment
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")
}

# Install Rust toolchain
function Install-Rust {
    if (Get-Command rustc -ErrorAction SilentlyContinue) {
        Write-Host ">>> Rust already installed: $(rustc --version)" -ForegroundColor Green
        Write-Host ">>> Updating Rust..."
        rustup update
        return
    }

    Write-Host ">>> Installing Rust toolchain..." -ForegroundColor Yellow

    # Download and run rustup-init
    $rustupInit = "$env:TEMP\rustup-init.exe"
    Invoke-WebRequest -Uri "https://win.rustup.rs/x86_64" -OutFile $rustupInit
    Start-Process -FilePath $rustupInit -ArgumentList "-y" -Wait -NoNewWindow

    # Add to PATH
    $cargoPath = "$env:USERPROFILE\.cargo\bin"
    $env:Path = "$cargoPath;$env:Path"
    [Environment]::SetEnvironmentVariable("Path", "$cargoPath;" + [Environment]::GetEnvironmentVariable("Path", "User"), "User")

    Write-Host ">>> Rust installed: $(rustc --version)" -ForegroundColor Green
}

# Install OpenCV via vcpkg
function Install-OpenCV-Vcpkg {
    Write-Host ">>> Installing OpenCV via vcpkg..." -ForegroundColor Yellow

    # Install vcpkg if not present
    if (-not (Test-Path "$VcpkgRoot\vcpkg.exe")) {
        Write-Host ">>> Installing vcpkg to $VcpkgRoot..." -ForegroundColor Yellow
        git clone https://github.com/Microsoft/vcpkg.git $VcpkgRoot
        & "$VcpkgRoot\bootstrap-vcpkg.bat"
    }

    # Install OpenCV
    & "$VcpkgRoot\vcpkg.exe" install opencv4:x64-windows

    # Integrate with system
    & "$VcpkgRoot\vcpkg.exe" integrate install

    # Set environment variable
    $vcpkgToolchain = "$VcpkgRoot\scripts\buildsystems\vcpkg.cmake"
    [Environment]::SetEnvironmentVariable("VCPKG_ROOT", $VcpkgRoot, "User")
    [Environment]::SetEnvironmentVariable("CMAKE_TOOLCHAIN_FILE", $vcpkgToolchain, "User")

    Write-Host ">>> OpenCV installed via vcpkg" -ForegroundColor Green
}

# Install OpenCV manually (prebuilt binaries)
function Install-OpenCV-Manual {
    Write-Host ">>> Installing OpenCV (prebuilt binaries)..." -ForegroundColor Yellow

    if (Test-Path "$OpenCVPath\build\OpenCVConfig.cmake") {
        Write-Host ">>> OpenCV already installed at $OpenCVPath" -ForegroundColor Green
        return
    }

    # Download OpenCV
    $opencvVersion = "4.9.0"
    $opencvZip = "$env:TEMP\opencv-$opencvVersion-windows.exe"
    $opencvUrl = "https://github.com/opencv/opencv/releases/download/$opencvVersion/opencv-$opencvVersion-windows.exe"

    Write-Host ">>> Downloading OpenCV $opencvVersion..."
    Invoke-WebRequest -Uri $opencvUrl -OutFile $opencvZip

    Write-Host ">>> Extracting OpenCV..."
    Start-Process -FilePath $opencvZip -ArgumentList "-o`"C:\`" -y" -Wait -NoNewWindow

    # Rename to standard location
    if (Test-Path "C:\opencv") {
        Remove-Item "C:\opencv" -Recurse -Force
    }
    Rename-Item "C:\opencv-$opencvVersion" $OpenCVPath

    # Set environment variables
    $opencvBin = "$OpenCVPath\build\x64\vc16\bin"
    $currentPath = [Environment]::GetEnvironmentVariable("Path", "User")
    if ($currentPath -notlike "*$opencvBin*") {
        [Environment]::SetEnvironmentVariable("Path", "$opencvBin;$currentPath", "User")
    }
    [Environment]::SetEnvironmentVariable("OpenCV_DIR", "$OpenCVPath\build", "User")

    Write-Host ">>> OpenCV installed at $OpenCVPath" -ForegroundColor Green
}

# Build and install Zenoh-C
function Install-ZenohC {
    Write-Host ">>> Installing Zenoh-C..." -ForegroundColor Yellow

    $buildDir = "$env:TEMP\zenoh-c-build"

    # Check if already installed
    if (Test-Path "$ZenohInstallPath\lib\cmake\zenohc") {
        Write-Host ">>> Zenoh-C already installed at $ZenohInstallPath" -ForegroundColor Green
        $response = Read-Host ">>> Reinstall? [y/N]"
        if ($response -ne "y" -and $response -ne "Y") {
            return
        }
    }

    # Clean up previous build
    if (Test-Path $buildDir) {
        Remove-Item $buildDir -Recurse -Force
    }
    New-Item -ItemType Directory -Path $buildDir | Out-Null

    # Clone Zenoh-C
    Write-Host ">>> Cloning Zenoh-C repository..."
    git clone --depth 1 https://github.com/eclipse-zenoh/zenoh-c.git "$buildDir\zenoh-c"

    Set-Location "$buildDir\zenoh-c"
    New-Item -ItemType Directory -Path "build" | Out-Null
    Set-Location "build"

    # Build with Visual Studio generator
    Write-Host ">>> Configuring Zenoh-C..."
    cmake .. -G "Visual Studio 17 2022" -A x64 `
        -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_INSTALL_PREFIX="$ZenohInstallPath"

    Write-Host ">>> Building Zenoh-C..."
    cmake --build . --config Release --parallel

    Write-Host ">>> Installing Zenoh-C..."
    cmake --install . --config Release

    # Add to PATH and CMAKE_PREFIX_PATH
    $zenohBin = "$ZenohInstallPath\bin"
    $currentPath = [Environment]::GetEnvironmentVariable("Path", "User")
    if ($currentPath -notlike "*$zenohBin*") {
        [Environment]::SetEnvironmentVariable("Path", "$zenohBin;$currentPath", "User")
    }

    $currentPrefixPath = [Environment]::GetEnvironmentVariable("CMAKE_PREFIX_PATH", "User")
    if (-not $currentPrefixPath) {
        [Environment]::SetEnvironmentVariable("CMAKE_PREFIX_PATH", $ZenohInstallPath, "User")
    }
    elseif ($currentPrefixPath -notlike "*$ZenohInstallPath*") {
        [Environment]::SetEnvironmentVariable("CMAKE_PREFIX_PATH", "$ZenohInstallPath;$currentPrefixPath", "User")
    }

    # Cleanup
    Set-Location $env:USERPROFILE
    Remove-Item $buildDir -Recurse -Force

    Write-Host ">>> Zenoh-C installed at $ZenohInstallPath" -ForegroundColor Green
}

# Verify installation
function Test-Installation {
    Write-Host ""
    Write-Host "=== Verifying Installation ===" -ForegroundColor Cyan

    # Refresh environment
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")

    $success = $true

    # Check CMake
    if (Get-Command cmake -ErrorAction SilentlyContinue) {
        Write-Host "[OK] CMake: $(cmake --version | Select-Object -First 1)" -ForegroundColor Green
    }
    else {
        Write-Host "[FAIL] CMake not found" -ForegroundColor Red
        $success = $false
    }

    # Check Ninja
    if (Get-Command ninja -ErrorAction SilentlyContinue) {
        Write-Host "[OK] Ninja: $(ninja --version)" -ForegroundColor Green
    }
    else {
        Write-Host "[FAIL] Ninja not found" -ForegroundColor Red
        $success = $false
    }

    # Check Rust
    if (Get-Command rustc -ErrorAction SilentlyContinue) {
        Write-Host "[OK] Rust: $(rustc --version)" -ForegroundColor Green
    }
    else {
        Write-Host "[FAIL] Rust not found" -ForegroundColor Red
        $success = $false
    }

    # Check OpenCV
    $opencvDir = [Environment]::GetEnvironmentVariable("OpenCV_DIR", "User")
    if ($opencvDir -and (Test-Path "$opencvDir\OpenCVConfig.cmake")) {
        Write-Host "[OK] OpenCV: $opencvDir" -ForegroundColor Green
    }
    elseif (Test-Path "$VcpkgRoot\installed\x64-windows\share\opencv4") {
        Write-Host "[OK] OpenCV: via vcpkg" -ForegroundColor Green
    }
    else {
        Write-Host "[FAIL] OpenCV not found" -ForegroundColor Red
        $success = $false
    }

    # Check Zenoh-C
    if (Test-Path "$ZenohInstallPath\lib\cmake\zenohc") {
        Write-Host "[OK] Zenoh-C: $ZenohInstallPath" -ForegroundColor Green
    }
    else {
        Write-Host "[FAIL] Zenoh-C not found" -ForegroundColor Red
        $success = $false
    }

    Write-Host ""
    if ($success) {
        Write-Host "=== All dependencies installed successfully! ===" -ForegroundColor Green
        Write-Host ""
        Write-Host "You can now build the project:" -ForegroundColor Cyan
        Write-Host "  cmake --preset windows-release"
        Write-Host "  cmake --build --preset windows-release"
        Write-Host ""
        Write-Host "Or with Ninja (from VS Developer Command Prompt):"
        Write-Host "  cmake --preset windows-ninja-release"
        Write-Host "  cmake --build --preset windows-ninja-release"
    }
    else {
        Write-Host "=== Some dependencies are missing. Please check the errors above. ===" -ForegroundColor Red
        exit 1
    }
}

# Main
function Main {
    if (-not (Test-Administrator)) {
        Write-Host "Warning: Not running as Administrator. Some installations may fail." -ForegroundColor Yellow
        Write-Host "Consider running this script as Administrator for system-wide installation." -ForegroundColor Yellow
        Write-Host ""
    }

    Install-Chocolatey
    Install-BasicTools
    Install-Rust

    if ($UseVcpkg) {
        Install-OpenCV-Vcpkg
    }
    else {
        Install-OpenCV-Manual
    }

    Install-ZenohC
    Test-Installation
}

Main
