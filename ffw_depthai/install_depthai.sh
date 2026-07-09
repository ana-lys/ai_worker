#!/bin/bash
set -e

echo "==========================================="
echo " Building depthai-core standalone via vcpkg"
echo "==========================================="

DEPTHAI_DIR="$HOME/depthai-core-build"
INSTALL_DIR="$HOME/depthai_install"

if [ ! -d "$DEPTHAI_DIR" ]; then
    echo "Cloning depthai-core..."
    git clone --recursive https://github.com/luxonis/depthai-core.git "$DEPTHAI_DIR"
fi

cd "$DEPTHAI_DIR"

echo "Configuring depthai-core..."
# Match default build settings - let vcpkg handle dependencies naturally
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Wno-psabi" \
    -DCMAKE_C_FLAGS="-Wno-psabi" \
    -DBUILD_SHARED_LIBS=ON \
    -DDEPTHAI_BUILD_EXAMPLES=OFF \
    -DDEPTHAI_BUILD_TESTS=OFF \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"

echo "Building depthai-core..."
cmake --build build --parallel $(nproc)

echo "Installing depthai-core to $INSTALL_DIR..."
cmake --install build

echo "==========================================="
echo " depthai-core successfully installed!"
echo " ROS 2 can now build instantly."
echo "==========================================="
