#!/usr/bin/env python3
"""
Find a joystick device by USB vendor/product ID or device name substring.

Outputs the device path (e.g. /dev/input/js2) on stdout, or exits with code 1
if no match is found.  Used by launch files so device mapping works regardless
of the order devices are enumerated on boot.

Usage:
  python3 find_joy_device.py --vendor 256f --product c63a   # SpaceMouse
  python3 find_joy_device.py --vendor 046d --product c215   # Logitech 3D Pro
  python3 find_joy_device.py --name "Logitech"              # name substring
  python3 find_joy_device.py --phys "1.4.4"                 # USB port match
  python3 find_joy_device.py -a                              # list all
"""
import argparse
import glob
import subprocess
import sys


def get_id(dev_path, attr):
    """Return the value of a single udev ATTR, or None."""
    try:
        out = subprocess.check_output(
            ['udevadm', 'info', '-a', '-n', dev_path],
            text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            stripped = line.strip()
            if f'ATTRS{{{attr}}}==' in stripped:
                return stripped.split('==')[1].strip('"')
    except Exception:
        pass
    return None


def list_all():
    """Print all joystick devices with their details."""
    for js in sorted(glob.glob('/dev/input/js*')):
        name = get_id(js, 'name') or '?'
        vendor = get_id(js, 'idVendor') or '?'
        product = get_id(js, 'idProduct') or '?'
        phys = get_id(js, 'phys') or '?'
        print(f'{js}  vendor={vendor}  product={product}  name="{name}"  phys={phys}')


def find(dev_paths, vendor=None, product=None, name=None, phys=None):
    """Return the first device matching all given criteria."""
    for js in sorted(dev_paths):
        ok = True
        if vendor:
            ok = ok and (get_id(js, 'idVendor') == vendor)
        if product:
            ok = ok and (get_id(js, 'idProduct') == product)
        if name:
            v = get_id(js, 'name') or ''
            ok = ok and (name.lower() in v.lower())
        if phys:
            v = get_id(js, 'phys') or ''
            ok = ok and (phys in v)
        if ok:
            return js
    return None


def main():
    p = argparse.ArgumentParser(description='Find a joystick device by properties.')
    p.add_argument('--vendor', help='USB vendor ID (e.g. 256f for 3Dconnexion)')
    p.add_argument('--product', help='USB product ID (e.g. c63a for SpaceMouse Wireless)')
    p.add_argument('--name', help='Substring match on device name')
    p.add_argument('--phys', help='Substring match on USB physical port')
    p.add_argument('-a', '--all', action='store_true', help='List all devices and exit')
    args = p.parse_args()

    if args.all:
        list_all()
        return

    if not any([args.vendor, args.product, args.name, args.phys]):
        p.print_help()
        sys.exit(1)

    js_devices = glob.glob('/dev/input/js*')
    if not js_devices:
        print('No joystick devices found.', file=sys.stderr)
        sys.exit(1)

    match = find(js_devices, vendor=args.vendor, product=args.product,
                 name=args.name, phys=args.phys)
    if match:
        print(match)
    else:
        print(f'No device matching the given criteria found.', file=sys.stderr)
        list_all()
        sys.exit(1)


if __name__ == '__main__':
    main()
