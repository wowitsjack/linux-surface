#!/bin/bash
set -e

MODULE_NAME="surface_s2idle_fix"

echo "=== Surface s2idle Death Sleep Fix ==="
echo ""

# Check if running on a Surface
if ! sudo dmesg 2>/dev/null | grep -qi "surface\|microsoft" && \
   ! cat /sys/class/dmi/id/product_name 2>/dev/null | grep -qi "surface"; then
    echo "WARNING: This doesn't appear to be a Surface device."
    echo "The module will refuse to load on non-Surface hardware anyway."
    echo ""
fi

# Build
echo "[1/4] Building module..."
make clean
make -j$(nproc)
echo "  OK"

# Install
echo "[2/4] Installing module..."
sudo make install
echo "  OK"

# Load
echo "[3/4] Loading module..."
sudo modprobe $MODULE_NAME || sudo insmod ${MODULE_NAME}.ko
echo "  OK"

# Auto-load on boot
echo "[4/4] Configuring auto-load on boot..."
echo "$MODULE_NAME" | sudo tee /etc/modules-load.d/${MODULE_NAME}.conf > /dev/null
echo "  OK"

echo ""
echo "=== Done! ==="
echo ""
sudo dmesg | grep $MODULE_NAME | tail -3
echo ""
echo "The module is loaded and will auto-load on boot."
echo "To uninstall: sudo ./uninstall.sh"
