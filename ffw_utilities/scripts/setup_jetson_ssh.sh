#!/bin/bash

# Ensure script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script as root (e.g., sudo ./setup_jetson_ssh.sh)"
  exit 1
fi

TARGET_USER="root"
TARGET_HOME="/root"

mkdir -p ${TARGET_HOME}/.ssh
chmod 700 ${TARGET_HOME}/.ssh

echo "Please paste your public SSH key (e.g. ssh-rsa AAAAB3Nza...) and press ENTER:"
read -r PUBLIC_KEY

if [ -z "$PUBLIC_KEY" ]; then
    echo "No key provided. Exiting."
    exit 1
fi

if ! grep -q "$PUBLIC_KEY" ${TARGET_HOME}/.ssh/authorized_keys 2>/dev/null; then
    echo "$PUBLIC_KEY" >> ${TARGET_HOME}/.ssh/authorized_keys
    chmod 600 ${TARGET_HOME}/.ssh/authorized_keys
    chown -R ${TARGET_USER}:${TARGET_USER} ${TARGET_HOME}/.ssh
    echo "Public key successfully added to ${TARGET_HOME}/.ssh/authorized_keys."
else
    echo "Public key already exists in authorized_keys."
fi

# SSH Server is already installed and configured on port 9999 in the container
echo "SSH Server should be ready. You can connect using: ssh -p 9999 ${TARGET_USER}@<jetson-ip>"
