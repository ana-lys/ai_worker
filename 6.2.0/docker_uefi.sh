#!/bin/bash

set -x

export TOP_NAME=6.2.0


if [ ! -d "../../${TOP_NAME}/uefi" ]; then
  mkdir ../../${TOP_NAME}/uefi
fi

if [ ! -d "../../${TOP_NAME}/uefi/patch" ]; then
  mkdir ../../${TOP_NAME}/uefi/patch
fi
sudo chmod 755 ./build_sdk/scripts/build_uefi.sh

cp -f ./build_sdk/patch/bootloader/*.bmp ../../${TOP_NAME}/uefi/patch
cp -f ./build_sdk/patch/bootloader/*.c ../../${TOP_NAME}/uefi/patch
cp -f ./build_sdk/scripts/build_uefi.sh ../../${TOP_NAME}/uefi/
cp -f ./build_sdk/scripts/rebuild_uefi.sh ../../${TOP_NAME}/uefi/

sudo docker run --rm \
    --net=host   --privileged \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="${PWD}/build_sdk:/home/robotis/build_sdk" \
    --volume="${PWD}/../../${TOP_NAME}:/home/robotis/${TOP_NAME}" \
    -e DISPLAY=$DISPLAY \
    -v ${HOME}/.Xauthority:/home/robotis/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w /home/robotis/${TOP_NAME}/uefi \
    jp${TOP_NAME}-robotis \
    ./build_uefi.sh 