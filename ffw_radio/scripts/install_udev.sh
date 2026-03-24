#!/bin/bash
# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RULE_PATH="$SCRIPT_DIR/../udev/99-radiolink.rules"

echo "Checking for RadioLink udev rules..."

if [ -f "$RULE_PATH" ]; then
    echo "Installing rules to /etc/udev/rules.d/..."
    sudo cp "$RULE_PATH" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Done! Please replug the RadioLink or check: ls -l /dev/input/radiolink_r16f"
else
    echo "Error: Could not find 99-radiolink.rules in $RULE_PATH"
    exit 1
fi
