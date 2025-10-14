#!/bin/bash

#set -x
set -e

export TOP_NAME=6.2.0


export BUILD_HOME=${PWD}

export TOP_DIR=${BUILD_HOME}/../${TOP_NAME}
export L4T_DIR=$TOP_DIR/Linux_for_Tegra
export LDK_ROOTFS_DIR=$TOP_DIR/Linux_for_Tegra/rootfs

export KERNEL_TOP=$TOP_DIR/kernel
export KERNEL_OUT=$KERNEL_TOP/kernel_out
export KERNEL_SRC=$KERNEL_TOP/kernel_src
export KERNEL_ORG=$KERNEL_TOP/kernel_org
export KERNEL_PKG_TOP=$KERNEL_TOP/kernel_pkg
export KERNEL_PKG_SRC=$KERNEL_PKG_TOP/Linux_for_Tegra/source

export CROSS_COMPILE=${TOP_DIR}/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export CROSS_COMPILE_AARCH64=${CROSS_COMPILE}
export CROSS_COMPILE_AARCH64_PATH=${TOP_DIR}/l4t-gcc/aarch64--glibc--stable-2022.08-1
export NV_TARGET_BOARD=t186ref



echo " "
echo "===== BUILD SDK ${TOP_NAME} for Orin ====="
echo "=                                         "
echo "=             2025. 02. 27                "
echo "=                                         "
echo "="
echo " "


# 필요한 프로그램 설치 
#
echo " "
echo "============= Install Program"
echo " "
echo " "
# sudo apt install apt-utils nano -y
# sudo apt install -y python3 
# sudo apt install -y python3-yaml 
# sudo apt install -y liblz4-tool cpio ssh 
# sudo apt install -y flex bison libssl-dev make device-tree-compiler python nano minicom
# sudo apt install -y wget xz-utils xxd flex bison build-essential bc flex bison
# sudo apt install -y qemu-user-static
# sudo apt install -y gcc-arm-linux-gnueabihf
# sudo apt install -y libxml2-utils 
# sudo apt install -y dosfstools
# sudo apt install -y libssl-dev


if [ ! -d "${TOP_DIR}" ]; then
  mkdir ${TOP_DIR}
fi
cd ${TOP_DIR}

if [ ! -d "${KERNEL_TOP}" ]; then
  mkdir ${KERNEL_TOP}
fi

if [ ! -d "${KERNEL_OUT}" ]; then
  mkdir ${KERNEL_OUT}
fi

if [ ! -d "${KERNEL_SRC}" ]; then
  mkdir ${KERNEL_SRC}
else
  rm -rf ${KERNEL_SRC}
  mkdir ${KERNEL_SRC}
fi

if [ ! -d "${KERNEL_ORG}" ]; then
  mkdir ${KERNEL_ORG}
fi

if [ ! -d "${KERNEL_PKG_TOP}" ]; then
  mkdir ${KERNEL_PKG_TOP}
fi


# 개발툴 설치
#
echo " "
echo "============= Install Develop Tools"
echo "="
echo "="
cd ${TOP_DIR}
if [ ! -d "${TOP_DIR}/l4t-gcc" ]; then
  mkdir ${TOP_DIR}/l4t-gcc
fi
cd ${TOP_DIR}/l4t-gcc

if [ ! -f "aarch64--glibc--stable-2022.08-1.tar.bz2" ]; then
  echo "wget aarch64--glibc--stable-2022.08-1.tar.bz2  "
  sudo wget \
      --no-verbose --show-progress \
      --progress=bar:force:noscroll \
      https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v3.0/toolchain/aarch64--glibc--stable-2022.08-1.tar.bz2      
fi 

if [ ! -d "${TOP_DIR}/l4t-gcc/aarch64--glibc--stable-2022.08-1" ]; then
  echo "tar xf aarch64--glibc--stable-2022.08-1.tar.bz2"
  tar xf aarch64--glibc--stable-2022.08-1.tar.bz2
fi


# JetPack BSP 다운로드
#
echo " "
echo "============= Download JetPack BSP"
echo "="
echo "="
cd ${TOP_DIR}
if [ ! -f "Jetson_Linux_r36.4.3_aarch64.tbz2" ]; then
  echo "wget Jetson_Linux_r36.4.3_aarch64.tbz2"
  wget \
    --no-verbose --show-progress \
    --progress=bar:force:noscroll \
    https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/Jetson_Linux_r36.4.3_aarch64.tbz2
fi

if [ ! -d "Linux_for_Tegra" ]; then
  echo "tar -vxjf Jetson_Linux_r36.4.3_aarch64.tbz2"
  tar -vxjf Jetson_Linux_r36.4.3_aarch64.tbz2
fi


# 파일시스템 다운로드 
#
echo " "
echo "============= Download Root File System"
echo "="
echo "="
if [ ! -f "Tegra_Linux_Sample-Root-Filesystem_r36.4.3_aarch64.tbz2" ]; then
  echo "wget Tegra_Linux_Sample-Root-Filesystem_r36.4.3_aarch64.tbz2"
  wget \
    --no-verbose --show-progress \
    --progress=bar:force:noscroll \
    https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/Tegra_Linux_Sample-Root-Filesystem_r36.4.3_aarch64.tbz2
fi 

if [ ! -d "Linux_for_Tegra/rootfs/boot" ]; then
  cd ${TOP_DIR}/Linux_for_Tegra/rootfs
  echo "sudo tar -jxpf ../../Tegra_Linux_Sample-Root-Filesystem_r36.4.3_aarch64.tbz2"
  sudo tar -jxpf ../../Tegra_Linux_Sample-Root-Filesystem_r36.4.3_aarch64.tbz2
fi
cd  ${TOP_DIR}/Linux_for_Tegra


# 파일시스템 바이너리 적용
#
echo " "
echo "============= Apply Binaries"
echo "="
echo "="
sudo ./apply_binaries.sh


# 필요한 소프트웨어 복사 
#
# echo " "
# echo "============= Copy Software"
# echo "="
# echo "="
# cd  ${BUILD_HOME}
# if [ -d "app" ]; then
# cp -f ./app/sdkmanager_1.9.0-10816_amd64.deb ${TOP_DIR}/sdkmanager_1.9.0-10816_amd64.deb
# sudo cp -f ./app/ZED_SDK_Tegra_L4T35.1_v3.7.7.run ${TOP_DIR}/Linux_for_Tegra/rootfs/home/
# sudo cp -f ./app/Wizard2-aarch64.AppImage.appimage ${TOP_DIR}/Linux_for_Tegra/rootfs/home/
# fi


# 리눅스 커널 소스 복사
#
echo " "
echo "============= Copy Kernel Source"
echo "="
echo "="
cd ${KERNEL_PKG_TOP}
if [ ! -f "public_sources.tbz2" ]; then
  echo "wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/sources/public_sources.tbz2"
  wget \
    --no-verbose --show-progress \
    --progress=bar:force:noscroll \
    https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/sources/public_sources.tbz2  
fi  

if [ ! -d "${KERNEL_PKG_SRC}" ]; then
  echo "tar -xjf public_sources.tbz2"
  tar -xjf public_sources.tbz2
fi

cd ${KERNEL_ORG}
if [ ! -d "${KERNEL_ORG}/kernel" ]; then
  echo "tar -xjf kernel_src.tbz2"
  tar -xjf ${KERNEL_PKG_SRC}/kernel_src.tbz2
fi  
echo "tar -xjf kernel_oot_modules_src.tbz2"
tar -xjf ${KERNEL_PKG_SRC}/kernel_oot_modules_src.tbz2 

echo "tar -xvf nvidia_kernel_display_driver_source.tbz2"
tar -xvf $KERNEL_PKG_SRC/nvidia_kernel_display_driver_source.tbz2 .


cd ${KERNEL_SRC}
if [ ! -d "${KERNEL_SRC}/kernel" ]; then
  echo "tar -xjf -xpf kernel_src.tbz2"
  tar -xjf ${KERNEL_PKG_SRC}/kernel_src.tbz2
fi  
echo "tar -xjf -xpf kernel_oot_modules_src.tbz2"
tar -xjf ${KERNEL_PKG_SRC}/kernel_oot_modules_src.tbz2

echo "tar -xvf nvidia_kernel_display_driver_source.tbz2"
tar -xvf $KERNEL_PKG_SRC/nvidia_kernel_display_driver_source.tbz2 .





# 패치 파일들 복사하기 
#
echo " "
echo "============= Copy Patch Files"
echo "="
echo "="
cd  ${TOP_DIR}/Linux_for_Tegra

cp ${BUILD_HOME}/scripts/*.sh ./
sudo chmod 755 build_kernel.sh
sudo chmod 755 flash_orin.sh
sudo chmod 755 docker_kernel_build.sh
sudo chmod 755 docker_flash_orin.sh


# 버전 추가 
#
sudo cp ${BUILD_HOME}/patch/ver_info.txt              ./rootfs/boot/


# 부트로더 변경
#
if [ ! -f "uefi_jetson-org.bin" ]; then
  cp ./bootloader/uefi_jetson.bin    ./bootloader/uefi_jetson-org.bin
fi  
cp -f ${BUILD_HOME}/patch/bootloader/uefi_jetson*.bin               ./bootloader/
cp -f ${BUILD_HOME}/patch/bootloader/uefi_jetson-robotis.bin        ./bootloader/uefi_jetson.bin

sudo cp ${BUILD_HOME}/patch/nvgetty.sh                ./rootfs/etc/systemd/
sudo cp ${BUILD_HOME}/patch/NVIDIA_Logo.png           ./rootfs/usr/share/backgrounds/
sudo cp ${BUILD_HOME}/patch/NVIDIA_Login_Logo.png     ./rootfs/usr/share/backgrounds/





# 파일 복사 
#
cd  ${L4T_DIR}
sudo cp ${BUILD_HOME}/patch/ssd_setup.tar.xz ./rootfs/home/


# Orin 패치 파일 복사

echo " "
echo "============= Apply Patch for Orin"
echo "="
echo "="
echo "= Copy Files"


#-- V02
#
cd ${L4T_DIR}
cp ${BUILD_HOME}/patch/r02/robotis-orin-r02.conf                                          ./
cp ${BUILD_HOME}/patch/r02/tegra234-mb1-bct-gpio-p3701-0000-a04-robotis-r02.dtsi          ./bootloader/
cp ${BUILD_HOME}/patch/r02/tegra234-mb1-bct-pinmux-p3701-0000-a04-robotis-r02.dtsi        ./bootloader/generic/BCT/
cp ${BUILD_HOME}/patch/r02/tegra234-mb2-bct-misc-p3701-0000-robotis-r02.dts               ./bootloader/generic/BCT/


cp ${BUILD_HOME}/patch/r02/dtb/Makefile                                                    ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/nv-platform/
cp ${BUILD_HOME}/patch/r02/overlay/Makefile                                                ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/overlay/
cp ${BUILD_HOME}/patch/r02/tegra234-p3701-0000-robotis-r02.dtsi                            ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/nv-platform/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-0000-robotis-r02.dtsi                            ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/nv-platform/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-0000+p3701-0000-dynamic-robotis-r02.dts          ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/overlay/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-0000+p3701-0000-robotis-r02.dts                  ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-0000+p3701-0004-nv-robotis-r02.dts               ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/nv-platform/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-0000+p3701-0004-robotis-r02.dts                  ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/staging/
cp ${BUILD_HOME}/patch/r02/defconfig                                                       ${KERNEL_SRC}/kernel/kernel-jammy-src/arch/arm64/configs/
cp ${BUILD_HOME}/patch/r02/tegra234-p3701-0000-robotis-r02-nv-public.dtsi                  ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/tegra234-p3701-0000-robotis-r02.dtsi


# 카메라 패치 
#
cp ${BUILD_HOME}/patch/r02/tegra234-camera-re26oc-robotis.dtsi                             ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/overlay/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-0000-camera-re26oc-robotis.dtsi                  ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/overlay/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-camera-modules-robotis.dtsi                      ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/overlay/
cp ${BUILD_HOME}/patch/r02/tegra234-p3737-camera-re26oc-overlay-robotis.dts                ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/overlay/
cp ${BUILD_HOME}/patch/r02/nv_imx185.c                                                     ${KERNEL_SRC}/nvidia-oot/drivers/media/i2c/
cp ${BUILD_HOME}/patch/r02/imx185_mode_tbls.h                                              ${KERNEL_SRC}/nvidia-oot/drivers/media/i2c/
cp ${BUILD_HOME}/patch/r02/tegra234-soc-camera-robotis.dtsi                                ${KERNEL_SRC}/hardware/nvidia/t23x/nv-public/nv-soc/tegra234-soc-camera.dtsi


# 리눅스 커널 빌드 
#
cd  ${L4T_DIR}
./build_kernel.sh


# OTA 툴 복사
#
echo " "
echo "============= Copy Tools for OTA"
echo "="
echo "="
sudo cp ${BUILD_HOME}/patch/ota/build_ota_payload.sh                ${L4T_DIR}/

sudo rm -r ${L4T_DIR}/tools/ota_tools
sudo cp -r ${BUILD_HOME}/patch/ota/ota_tools                        ${L4T_DIR}/tools/


echo " "
echo "============= End" 