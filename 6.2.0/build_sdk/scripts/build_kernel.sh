#!/bin/bash

set -x -e

export BUILD_HOME=${PWD}

export TOP_DIR=${BUILD_HOME}/..
export L4T_DIR=$TOP_DIR/Linux_for_Tegra
export LDK_ROOTFS_DIR=$TOP_DIR/Linux_for_Tegra/rootfs

export KERNEL_TOP=$TOP_DIR/kernel
export KERNEL_OUT=$KERNEL_TOP/kernel_out
export KERNEL_SRC=$KERNEL_TOP/kernel_src
export KERNEL_ORG=$KERNEL_TOP/kernel_org
export KERNEL_PKG_TOP=$KERNEL_TOP/kernel_pkg
export KERNEL_PKG_SRC=$KERNEL_PKG_TOP/Linux_for_Tegra/source
export KERNEL_DSP=$KERNEL_TOP/kernel_dsp

export CROSS_COMPILE=${TOP_DIR}/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export CROSS_COMPILE_AARCH64=${CROSS_COMPILE}
export CROSS_COMPILE_AARCH64_PATH=${TOP_DIR}/l4t-gcc/aarch64--glibc--stable-2022.08-1
export NV_TARGET_BOARD=t186ref
export ARCH=arm64
export LOCALVERSION="-tegra"
export IGNORE_PREEMPT_RT_PRESENCE=1
export KERNEL_HEADERS=$KERNEL_SRC/kernel/kernel-jammy-src
export INSTALL_MOD_PATH=$LDK_ROOTFS_DIR


# 커널 소스 빌드
#
echo "2025-02-27-R1"
echo " "
echo "============= Begin : Build Linux Kernel"
echo "="
echo "="
cd ${KERNEL_SRC}
./nvbuild.sh -o ${KERNEL_OUT}
echo "============= End : Build Linux Kernel"


# 커널 모듈 설치
#
echo " "
echo "============= Begin : Install Kernel Modules"
echo "="
echo "="
cd ${KERNEL_SRC}
./nvbuild.sh -i -o ${KERNEL_OUT}
echo "============= End : Install Kernel Modules"


# 디바이스트리, 커널 복사
#
echo " "
echo "============= Begin : Copy DTB, Kernel"
echo "="
echo "="
cd  ${L4T_DIR}
cp ${KERNEL_OUT}/kernel-devicetree/generic-dts/dtbs/*               ${L4T_DIR}/kernel/dtb/
cp ${KERNEL_OUT}/kernel/kernel-jammy-src/arch/arm64/boot/Image      ${L4T_DIR}/kernel/

echo " "
echo "============= End : Copy DTB, Kernel"


echo " "
echo "============= End"


