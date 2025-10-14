#!/bin/bash

#set -e

export ROOTFS_USER="robotis"
export ROOTFS_PASS="root"
export BOARD_VER="empty"
export IMAGE_REUSE="false"
export IMAGE_FLAG=""
export ROOTFS_GROUP="orin-jp620"

while getopts u:p:v:r:g: flag
do
    case "${flag}" in
        u) ROOTFS_USER=${OPTARG};;
        p) ROOTFS_PASS=${OPTARG};;
        v) BOARD_VER=${OPTARG};;
        r) IMAGE_REUSE=${OPTARG};;
        g) ROOTFS_GROUP=${OPTARG};;
    esac
done

if [ ${IMAGE_REUSE} == "true" ]; then
  export IMAGE_FLAG="-r"
fi


if [ ${BOARD_VER} == "empty" ]
  then
    cat ./rootfs/boot/ver_info.txt
    echo "Option"
    echo "  -v [v02]"
    echo "  -u rootfs_user"
    echo "  -p rootfs_passwd"
    echo "  -r [true|false]"
    echo "  -g rootfs_group"
    exit 1
  else
    echo "B/D VER     : "${BOARD_VER}
    echo "ROOTFS USER : "${ROOTFS_USER}
    echo "ROOTFS PASS : "${ROOTFS_PASS}
    echo "IMAGE_REUSE : "${IMAGE_REUSE}
    echo "IMAGE_FLAG  : "${IMAGE_FLAG}
    echo "ROOTFS GROUP: "${ROOTFS_GROUP}
fi


if [ -f "./rootfs/boot/ver_board.txt" ]; then
  sudo rm ./rootfs/boot/ver_board.txt
fi 

if [ ${BOARD_VER} == "v02" ]; then
  echo "ORIN BD V0.2" | sudo tee -a ./rootfs/boot/ver_board.txt
  sudo ./tools/l4t_create_default_user.sh -u ${ROOTFS_USER} -p ${ROOTFS_PASS} --accept-license -a -n ${ROOTFS_GROUP}
  sudo ./flash.sh ${IMAGE_FLAG} robotis-orin-r02 mmcblk0p1
fi