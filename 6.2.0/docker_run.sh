#!/bin/bash

set -x

export TOP_NAME=6.2.0

sudo chmod 755 ./build_sdk/build_sdk.sh

sudo docker run -it --rm \
    --net=host   --privileged \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="${PWD}/build_sdk:/home/robotis/build_sdk" \
    --volume="${PWD}/../../${TOP_NAME}:/home/robotis/${TOP_NAME}" \
    -e DISPLAY=$DISPLAY \
    -v ${HOME}/.Xauthority:/home/robotis/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w /home/robotis/${TOP_NAME}/Linux_for_Tegra \
    jp${TOP_NAME}-robotis 