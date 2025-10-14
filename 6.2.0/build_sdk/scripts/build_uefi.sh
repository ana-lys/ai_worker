#!/bin/bash

set -x -e

sudo apt update

if [ -z "${USER}"]; then
export USER=robotis
fi

export UEFI_HOME=${PWD}

# 폴더 생성 
if [ ! -d "uefi" ]; then
  mkdir uefi
fi
cd uefi


# edkrepo 설치
cd ${UEFI_HOME}/uefi
#sudo apt-get -y install git python3 python3-setuptools python3-pip

if [ ! -d "edkrepo" ]; then
  mkdir edkrepo
fi  
cd ${UEFI_HOME}/uefi/edkrepo

if [ ! -f "edkrepo-3.2.4.0.tar.gz" ]; then
  wget https://github.com/tianocore/edk2-edkrepo/releases/download/edkrepo-v3.2.4/edkrepo-3.2.4.0.tar.gz
fi
tar xvf edkrepo-3.2.4.0.tar.gz
sudo ./install.py --system --no-prompt --user ${USER}

cd ..
sudo chown -R ${USER}. ~/.edkrepo
edkrepo manifest-repos add nvidia https://github.com/NVIDIA/edk2-edkrepo-manifest.git main nvidia


# Create workspace
if [ ! -d "nvidia-uefi" ]; then
  edkrepo clone nvidia-uefi NVIDIA-Platforms uefi-202502.0
fi
cd nvidia-uefi


# 로고파일 복사
echo " "
echo "Copy Boot Logo"
cp -f ${UEFI_HOME}/patch/*.bmp ${UEFI_HOME}/uefi/nvidia-uefi/edk2-nvidia/Silicon/NVIDIA/Assets/

# 패치파일 복사
cp -f ${UEFI_HOME}/patch/BmBoot.c       ${UEFI_HOME}/uefi/nvidia-uefi/edk2/MdeModulePkg/Library/UefiBootManagerLib/BmBoot.c
cp -f ${UEFI_HOME}/patch/BootLogoLib.c  ${UEFI_HOME}/uefi/nvidia-uefi/edk2/MdeModulePkg/Library/BootLogoLib/BootLogoLib.c
cp -f ${UEFI_HOME}/patch/PlatformBm.c   ${UEFI_HOME}/uefi/nvidia-uefi/edk2-nvidia/Silicon/NVIDIA/Library/PlatformBootManagerLib/PlatformBm.c


# 빌드하기
sudo apt install -y build-essential uuid-dev git gcc python3 virtualenv gcc-aarch64-linux-gnu
sudo apt install -y device-tree-compiler

./edk2-nvidia/Platform/NVIDIA/Jetson/build.sh