#!/bin/bash

set -x

export TOP_NAME=6.2.0

sudo docker run --rm \
    --net=host   --privileged \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="${PWD}/../../${TOP_NAME}:/home/robotis/${TOP_NAME}" \
    -w /home/robotis/${TOP_NAME}/Linux_for_Tegra \
    jp${TOP_NAME}-robotis \
    ./build_kernel.sh 