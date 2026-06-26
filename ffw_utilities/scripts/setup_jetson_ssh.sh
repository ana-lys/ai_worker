#!/bin/bash

# Ensure script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script as root (e.g., sudo ./setup_jetson_ssh.sh)"
  exit 1
fi

echo "Installing openssh-server..."
apt-get update
apt-get install -y openssh-server

echo "Configuring SSH port to 9999..."
# Change Port 22 to 9999, or append if not present
if grep -q "^#Port 22" /etc/ssh/sshd_config; then
  sed -i 's/^#Port 22/Port 9999/' /etc/ssh/sshd_config
elif grep -q "^Port " /etc/ssh/sshd_config; then
  sed -i 's/^Port .*/Port 9999/' /etc/ssh/sshd_config
else
  echo "Port 9999" >> /etc/ssh/sshd_config
fi

echo "Setting up authorized_keys for root..."
TARGET_USER="root"
TARGET_HOME="/root"

mkdir -p ${TARGET_HOME}/.ssh
chmod 700 ${TARGET_HOME}/.ssh

# Replace this string with your actual public key (e.g., "ssh-ed25519 AAAAC3NzaC... user@host")
# NOTE: It is completely cryptographically safe to share your public key. However, it's 
# good practice not to commit it into public version control just to avoid hardcoding.
PUBLIC_KEY="ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQC... DUMMY_KEY_PLEASE_REPLACE"

if [ "$PUBLIC_KEY" != "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQC... DUMMY_KEY_PLEASE_REPLACE" ]; then
    if ! grep -q "$PUBLIC_KEY" ${TARGET_HOME}/.ssh/authorized_keys 2>/dev/null; then
        echo "$PUBLIC_KEY" >> ${TARGET_HOME}/.ssh/authorized_keys
        chmod 600 ${TARGET_HOME}/.ssh/authorized_keys
        chown -R ${TARGET_USER}:${TARGET_USER} ${TARGET_HOME}/.ssh
        echo "Public key added to ${TARGET_HOME}/.ssh/authorized_keys."
    else
        echo "Public key already exists in authorized_keys."
    fi
else
    echo "Skipping public key addition (dummy key detected). Please edit the script to add your real key."
fi

echo "Restarting SSH service..."
systemctl restart ssh

echo "SSH Server setup complete!"
echo "You can connect using: ssh -p 9999 ${TARGET_USER}@<jetson-ip>"
