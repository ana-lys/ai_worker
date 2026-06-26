#!/bin/bash

CONTAINER_NAME="ai_worker"

if ! docker ps | grep -q "$CONTAINER_NAME"; then
    echo "Error: Container is not running. Please run container.sh start first."
    exit 1
fi

echo "Please paste your public SSH key (e.g. ssh-rsa AAAAB3Nza...) and press ENTER:"
read -r PUBLIC_KEY

if [ -z "$PUBLIC_KEY" ]; then
    echo "No key provided. Exiting."
    exit 1
fi

docker exec -u root "$CONTAINER_NAME" bash -c "mkdir -p /root/.ssh && chmod 700 /root/.ssh && if ! grep -q '$PUBLIC_KEY' /root/.ssh/authorized_keys 2>/dev/null; then echo '$PUBLIC_KEY' >> /root/.ssh/authorized_keys; chmod 600 /root/.ssh/authorized_keys; echo 'Public key successfully added to container.'; else echo 'Public key already exists in container.'; fi"

echo "You can now connect to the container using: ssh -p 9999 root@<jetson-ip>"
