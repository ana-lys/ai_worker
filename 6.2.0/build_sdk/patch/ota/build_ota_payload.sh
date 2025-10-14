#!/bin/bash


export BASE_BSP=/home/embed2/work/orin/6.0.0/Linux_for_Tegra


# OTA Payload 생성
# 6.0.0 -> 6.2.0용 payload 
#
echo " "
echo "============= Generate OTA Payload (6.0.0 -> 6.2.0)"
echo "="
echo "="
sudo chmod 777 ./tools/ota_tools/version_upgrade/l4t_generate_ota_package.sh
sudo ROOTFS_AB=0 -E ./tools/ota_tools/version_upgrade/l4t_generate_ota_package.sh -s robotis-orin-r02 R36-3

echo " "
echo "payload path : Linux_for_Tegra/bootloader/robotis-orin-r02"
echo " "
echo "============= End"