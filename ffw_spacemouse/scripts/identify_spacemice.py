#!/usr/bin/env python3
import glob
import subprocess

def get_udev_info(dev_path):
    try:
        output = subprocess.check_output(['udevadm', 'info', '-a', '-n', dev_path], text=True, stderr=subprocess.DEVNULL)
        info = {'name': 'Unknown', 'serials': [], 'uniqs': [], 'phys': 'N/A', 'idVendor': 'N/A', 'idProduct': 'N/A'}
        
        for line in output.splitlines():
            line = line.strip()
            if 'ATTRS{name}==' in line and info['name'] == 'Unknown':
                info['name'] = line.split('==')[1].strip('"')
            if 'ATTRS{serial}==' in line:
                s = line.split('==')[1].strip('"')
                if s not in info['serials']: info['serials'].append(s)
            if 'ATTRS{uniq}==' in line:
                u = line.split('==')[1].strip('"')
                if u and u not in info['uniqs']: info['uniqs'].append(u)
            if 'ATTRS{phys}==' in line and info['phys'] == 'N/A':
                info['phys'] = line.split('==')[1].strip('"')
            if 'ATTRS{idVendor}==' in line and info['idVendor'] == 'N/A':
                info['idVendor'] = line.split('==')[1].strip('"')
            if 'ATTRS{idProduct}==' in line and info['idProduct'] == 'N/A':
                info['idProduct'] = line.split('==')[1].strip('"')
        return info
    except Exception as e:
        return None

def main():
    print("SpaceMouse Identification Tool")
    print("------------------------------")
    
    js_devices = glob.glob('/dev/input/js*')
    if not js_devices:
        print("No joysticks found in /dev/input/js*")
        return

    found = False
    for dev in sorted(js_devices):
        info = get_udev_info(dev)
        if not info or '3Dconnexion' not in info.get('name', ''):
            continue
            
        found = True
        print(f"\nDevice Node: {dev}")
        print(f"Name:        {info['name']}")
        print(f"Vendor:      {info['idVendor']}")
        print(f"Product:     {info['idProduct']}")
        print(f"All Serials: {info['serials']}")
        print(f"All Uniqs:   {info['uniqs']}")
        print(f"Phys Port:   {info['phys']}")
        
        # Cross check with lsusb
        if info['idVendor'] != 'N/A':
            print("--- LSUSB Info ---")
            try:
                lsusb_out = subprocess.check_output(f"lsusb -d {info['idVendor']}:{info['idProduct']} -v | grep iSerial", shell=True, text=True, stderr=subprocess.DEVNULL)
                print(lsusb_out.strip())
            except:
                print("No iSerial descriptor found via lsusb.")
        
    if not found:
        print("No 3Dconnexion SpaceMouse devices detected.")
        
if __name__ == '__main__':
    main()
