#!/usr/bin/env python3
import struct
import os
import time

def read_js_events(device_path):
    print(f"Opening {device_path}...")
    try:
        with open(device_path, 'rb') as f:
            print(f"Successfully connected to {device_path}!")
            print("Press buttons to see their state. Press Ctrl+C to exit.\n")
            while True:
                # 8 bytes per event: time (4), value (2), type (1), number (1)
                event_bytes = f.read(8)
                if event_bytes:
                    tv_msec, value, type_, number = struct.unpack('IhBB', event_bytes)
                    
                    if type_ & 0x01: # Button event
                        state = "PRESSED" if value == 1 else "RELEASED"
                        print(f"[{device_path}] Button {number}: {state}")
                else:
                    break
    except FileNotFoundError:
        print(f"Error: {device_path} not found.")
    except Exception as e:
        print(f"Error reading {device_path}: {e}")

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 test_buttons.py /dev/input/jsX")
        print("Example: python3 test_buttons.py /dev/input/js0")
        sys.exit(1)
    
    read_js_events(sys.argv[1])
