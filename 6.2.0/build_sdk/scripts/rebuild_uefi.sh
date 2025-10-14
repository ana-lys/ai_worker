#!/bin/bash

set -x -e


if [ -z "${USER}"]; then
export USER=robotis
fi

export UEFI_HOME=${PWD}

cd ${UEFI_HOME}/uefi/nvidia-uefi

sudo apt update

# 빌드하기
sudo apt install -y build-essential uuid-dev git gcc python3 virtualenv gcc-aarch64-linux-gnu
sudo apt install -y device-tree-compiler


./edk2-nvidia/Platform/NVIDIA/Jetson/build.sh

