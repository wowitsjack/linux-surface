#!/bin/bash
# Quick-deploy surface_s2idle_fix after fresh OS install
# Surface Laptop 5 - kernel 6.18.7-surface-1
#
# Prerequisites:
#   1. Install linux-surface kernel:
#      wget -qO - https://raw.githubusercontent.com/linux-surface/linux-surface/master/pkg/keys/surface.asc \
#        | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/linux-surface.gpg > /dev/null
#      echo "deb [arch=amd64] https://pkg.surfacelinux.com/debian release main" \
#        | sudo tee /etc/apt/sources.list.d/linux-surface.list
#      sudo apt update
#      sudo apt install linux-image-surface linux-headers-surface
#      sudo reboot
#
#   2. After reboot on surface kernel, run this script.

set -e

echo "=== Surface s2idle fix deploy ==="

# Check we're on the right kernel
KVER=$(uname -r)
echo "Kernel: $KVER"

# Install build deps if needed
if ! dpkg -l | grep -q "linux-headers-$KVER"; then
    echo "Installing kernel headers..."
    sudo apt install -y "linux-headers-$KVER"
fi

if ! command -v make &>/dev/null; then
    echo "Installing build-essential..."
    sudo apt install -y build-essential
fi

# Build
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
echo "Building from $SCRIPT_DIR ..."
cd "$SCRIPT_DIR"
make clean
make

# Install
sudo make install

# Autoload config
echo "Setting up autoload..."
echo -e "surface_gpe\nsurface_s2idle_fix" | sudo tee /etc/modules-load.d/surface_s2idle_fix.conf

# Update initramfs so the module is baked in
sudo update-initramfs -u -k "$KVER"

# Load it now
sudo modprobe surface_s2idle_fix

echo ""
echo "=== Done! ==="
echo "Module loaded. Check with: sudo dmesg | grep surface_s2idle"
echo "Close the lid to test sleep. Open lid should wake."
