#!/bin/bash

set -x

export TOP_NAME=6.2.0


if [ ! -d "../../${TOP_NAME}" ]; then
  mkdir ../../${TOP_NAME}
fi  
sudo docker build -t jp${TOP_NAME}-robotis  .

sudo chmod 755 ./build_sdk/build_sdk.sh

sudo docker run --rm \
    --net=host   --privileged \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="${PWD}/build_sdk:/home/robotis/build_sdk" \
    --volume="${PWD}/../../${TOP_NAME}:/home/robotis/${TOP_NAME}" \
    -e DISPLAY=$DISPLAY \
    -v ${HOME}/.Xauthority:/home/robotis/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w /home/robotis/build_sdk \
    jp${TOP_NAME}-robotis \
    ./build_sdk.sh $1 $2