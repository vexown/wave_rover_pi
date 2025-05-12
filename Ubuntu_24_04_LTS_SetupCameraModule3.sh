#!/usr/bin/env bash
# Automated setup script for Raspberry Pi Camera Module 3 (IMX708)
# on Ubuntu Server 24.04.x (64-bit) running on Raspberry Pi 4B.

set -euo pipefail

# Variables
LIBCAMERA_REPO="https://github.com/raspberrypi/libcamera.git"
RPICAM_APPS_REPO="https://github.com/raspberrypi/rpicam-apps.git"
BUILD_DIR="/home/$USER/libcamera_build"
NUM_CORES=$(nproc)

# Step 1: Update and install dependencies
echo "[*] Updating system and installing build dependencies..."
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y \
  git meson ninja-build \
  libboost-dev libdrm-dev libexpat1-dev libjpeg-dev libpng-dev \
  libssl-dev pkg-config cmake \
  python3-pip

# Step 2: Prepare build directory
echo "[*] Preparing build directory at $BUILD_DIR..."
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Step 3: Clone and build libcamera
echo "[*] Cloning libcamera repository..."
git clone "$LIBCAMERA_REPO" libcamera
cd libcamera
mkdir -p build && cd build

echo "[*] Configuring libcamera with Meson..."
meson setup --prefix=/usr ..

echo "[*] Building libcamera (this may take a while)..."
ninja -j "$NUM_CORES"

echo "[*] Installing libcamera..."
sudo ninja install
cd "$BUILD_DIR"

# Step 4: Clone and build rpicam-apps
echo "[*] Cloning rpicam-apps repository..."
git clone "$RPICAM_APPS_REPO" rpicam-apps
cd rpicam-apps\mkdir -p build && cd build

echo "[*] Configuring rpicam-apps with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "[*] Building rpicam-apps..."
make -j "$NUM_CORES"

echo "[*] Installing rpicam-apps..."
sudo make install

# Step 5: Finalize
echo "[*] Setup complete!"
echo "You can now test the camera with: libcamera-hello"
