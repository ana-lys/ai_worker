#!/bin/bash

CONTAINER_NAME="ai_worker"

if ! docker ps | grep -q "$CONTAINER_NAME"; then
    echo "Error: Container is not running. Please run container.sh start first."
    exit 1
fi

PUBLIC_KEY=""
DEFAULT_KEY_FILE="$HOME/.ssh/ffw_container.pub"

# Automatically try to get the public key from the default file
if [ -f "$DEFAULT_KEY_FILE" ]; then
    PUBLIC_KEY=$(cat "$DEFAULT_KEY_FILE")
    if [ -n "$PUBLIC_KEY" ]; then
        echo "Automatically loaded public key from $DEFAULT_KEY_FILE"
    fi
fi

# If not found or file was empty, query the user
if [ -z "$PUBLIC_KEY" ]; then
    echo "Please paste your public SSH key (e.g. ssh-rsa AAAAB3Nza...) and press ENTER:"
    read -r PUBLIC_KEY
fi

# If still blank, skip smoothly without error
if [ -z "$PUBLIC_KEY" ]; then
    echo "No key provided. Skipping SSH key setup."
    exit 0
fi

docker exec -u root "$CONTAINER_NAME" bash -c "mkdir -p /root/.ssh && chmod 700 /root/.ssh && if ! grep -q '$PUBLIC_KEY' /root/.ssh/authorized_keys 2>/dev/null; then echo '$PUBLIC_KEY' >> /root/.ssh/authorized_keys; chmod 600 /root/.ssh/authorized_keys; echo 'Public key successfully added to container.'; else echo 'Public key already exists in container.'; fi"

echo "You can now connect to the container using: ssh -p 9999 root@<jetson-ip>"
