#!/bin/bash
set -e

MODULE_NAME="surface_s2idle_fix"

echo "=== Uninstalling Surface s2idle Fix ==="

# Unload
if lsmod | grep -q $MODULE_NAME; then
    echo "Unloading module..."
    sudo rmmod $MODULE_NAME
fi

# Remove auto-load
if [ -f /etc/modules-load.d/${MODULE_NAME}.conf ]; then
    echo "Removing auto-load config..."
    sudo rm /etc/modules-load.d/${MODULE_NAME}.conf
fi

# Remove installed module
KVER=$(uname -r)
MOD_PATH="/lib/modules/${KVER}/extra/${MODULE_NAME}.ko"
if [ -f "$MOD_PATH" ] || [ -f "${MOD_PATH}.zst" ] || [ -f "${MOD_PATH}.xz" ]; then
    echo "Removing installed module..."
    sudo rm -f "$MOD_PATH" "${MOD_PATH}.zst" "${MOD_PATH}.xz"
    sudo depmod -a
fi

echo ""
echo "=== Uninstalled ==="
