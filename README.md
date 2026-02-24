# surface-s2idle-fix

Kernel module that fixes the s2idle "death sleep" bug on the Microsoft Surface Laptop 5 running Linux.

Without this module, closing the lid (or any s2idle suspend) results in permanent sleep that requires a 10-second power button hold to recover.

## What it does

Masks ACPI GPE 0x52 (lid switch) during s2idle to prevent a spurious interrupt from corrupted GPIO pad configuration. On resume, restores the pad config, unmasks the GPE, and forces a lid state resync so logind sees the correct lid position.

## The bug

The Intel INTC1055 pin controller power-gates its pad communities during s2idle idle states. This corrupts the PADCFG0 RXINV (receive invert) bit on pin 213 (the lid switch GPIO). The corrupted bit generates a false edge on GPE 0x52, which fires an SCI. The kernel's s2idle wake handler treats this as a spurious event and calls `pm_system_cancel_wakeup()`, permanently poisoning the wakeup framework. After that, no legitimate wake source (RTC alarm, power button) can wake the system.

## Tested on

- Microsoft Surface Laptop 5 (Intel 12th Gen Alder Lake-U)
- Arch Linux, kernel 6.18.8-arch2-1-surface (linux-surface)
- COSMIC desktop environment

7/7 suspend/resume cycles survived with the module loaded. 0/7 survived without it.

## Install

```bash
# Quick install (builds, installs, auto-loads on boot)
chmod +x install.sh
./install.sh
```

### Manual install

```bash
make
sudo insmod surface_s2idle_fix.ko
```

### DKMS install (survives kernel updates)

```bash
sudo cp -r . /usr/src/surface-s2idle-fix-1.0.0
sudo dkms add surface-s2idle-fix/1.0.0
sudo dkms build surface-s2idle-fix/1.0.0
sudo dkms install surface-s2idle-fix/1.0.0
echo "surface_s2idle_fix" | sudo tee /etc/modules-load.d/surface_s2idle_fix.conf
```

## Uninstall

```bash
chmod +x uninstall.sh
./uninstall.sh
```

## Verify it's working

```bash
# Check module is loaded
lsmod | grep surface_s2idle_fix

# Check dmesg for module messages
sudo dmesg | grep surface_s2idle_fix
```

You should see something like:
```
surface_s2idle_fix: loaded: SCI=9 PADCFG0=0x40080100 RXINV=0
surface_s2idle_fix:   GPE 0x52 will be masked during s2idle to prevent spurious wake death spiral
```

After a suspend/resume cycle:
```
surface_s2idle_fix: suspend #1: masked GPE 0x52, PADCFG0=0x40080100 RXINV=0
surface_s2idle_fix: resume #1: lid state resynced via \_GPE._L52
surface_s2idle_fix: resume #1: unmasked+enabled GPE 0x52, handler_calls=0 padcfg_restores=0 lid_resyncs=1
```

## Known limitations

- **Lid open does not wake from sleep.** GPE 0x52 is masked during s2idle, so opening the lid won't wake the system. Use the power button to wake. This is a deliberate trade-off vs. permanent death sleep.
- Only tested on Surface Laptop 5. May work on other Surface devices with the same Intel INTC1055 pin controller, but the PADCFG physical addresses are hardcoded and would need adjusting.

## License

GPL-2.0
