#!/usr/bin/env python3
"""
Identify joystick devices connected to the system.
Detects both 3Dconnexion SpaceMouse and Logitech G Extreme 3D Pro hardware.
"""
import glob
import subprocess


def get_udev_info(dev_path):
    try:
        output = subprocess.check_output(['udevadm', 'info', '-a', '-n', dev_path], text=True, stderr=subprocess.DEVNULL)
        info = {
            'name': 'Unknown', 'serials': [], 'uniqs': [],
            'phys': 'N/A', 'idVendor': 'N/A', 'idProduct': 'N/A',
        }

        for line in output.splitlines():
            line = line.strip()
            if 'ATTRS{name}==' in line and info['name'] == 'Unknown':
                info['name'] = line.split('==')[1].strip('"')
            if 'ATTRS{serial}==' in line:
                s = line.split('==')[1].strip('"')
                if s and s not in info['serials']:
                    info['serials'].append(s)
            if 'ATTRS{uniq}==' in line:
                u = line.split('==')[1].strip('"')
                if u and u not in info['uniqs']:
                    info['uniqs'].append(u)
            if 'ATTRS{phys}==' in line and info['phys'] == 'N/A':
                info['phys'] = line.split('==')[1].strip('"')
            if 'ATTRS{idVendor}==' in line and info['idVendor'] == 'N/A':
                info['idVendor'] = line.split('==')[1].strip('"')
            if 'ATTRS{idProduct}==' in line and info['idProduct'] == 'N/A':
                info['idProduct'] = line.split('==')[1].strip('"')
        return info
    except Exception as e:
        return None


def classify_device(info):
    """Classify a joystick device by name/vendor."""
    name = info.get('name', '')
    vendor = info.get('idVendor', '')
    product = info.get('idProduct', '')

    if '3Dconnexion' in name:
        return 'SpaceMouse'
    if vendor == '046d' or 'Logitech' in name:
        return 'Logitech Gamepad / Joystick'
    return 'Unknown'


def main():
    print("Joystick Identification Tool")
    print("----------------------------")

    js_devices = glob.glob('/dev/input/js*')
    if not js_devices:
        print("No joystick devices found in /dev/input/js*")
        print("Make sure your device is plugged in.")
        return

    found = False
    for dev in sorted(js_devices):
        info = get_udev_info(dev)
        if not info:
            print(f"\n[!] Could not read udev info for {dev}")
            continue

        found = True
        device_type = classify_device(info)

        print(f"\nDevice Node:   {dev}")
        print(f"Type:          {device_type}")
        print(f"Name:          {info['name']}")
        print(f"Vendor:        {info['idVendor']}")
        print(f"Product:       {info['idProduct']}")
        print(f"All Serials:   {info['serials']}")
        print(f"All Uniqs:     {info['uniqs']}")
        print(f"Phys Port:     {info['phys']}")

        # Cross check with lsusb
        if info['idVendor'] != 'N/A':
            print("  --- LSUSB Info ---")
            try:
                lsusb_out = subprocess.check_output(
                    f"lsusb -d {info['idVendor']}:{info['idProduct']} -v | grep iSerial",
                    shell=True, text=True, stderr=subprocess.DEVNULL,
                )
                for line in lsusb_out.strip().splitlines():
                    print(f"  {line.strip()}")
            except Exception:
                print("  No iSerial descriptor found via lsusb.")

    if not found:
        print("No joystick devices detected.")

    print("\n----------------------------")
    print("To test buttons interactively:")
    print("  python3 identify_joysticks.py /dev/input/jsX")
    print("  (replacing jsX with the device node shown above)")
    print("")
    print("To configure persistent device symlinks, copy udev_rules.txt")
    print("to /etc/udev/rules.d/ after replacing the phys/serial with")
    print("the values shown above, then run:")
    print("  sudo udevadm control --reload-rules && sudo udevadm trigger")


if __name__ == '__main__':
    main()
