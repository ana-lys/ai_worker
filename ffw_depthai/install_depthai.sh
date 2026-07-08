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

# Enable vcpkg binary caching (using GitHub packages or default)
export VCPKG_BINARY_SOURCES="clear;default,readwrite"

echo "Configuring depthai-core..."
# We disable heavy features to ensure even if it compiles from source, it takes seconds, not hours.
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DDEPTHAI_BUILD_EXAMPLES=OFF \
    -DDEPTHAI_BUILD_TESTS=OFF \
    -DDEPTHAI_OPENCV_USE_SYSTEM=ON \
    -DDEPTHAI_ENABLE_CURL=OFF \
    -DDEPTHAI_ENABLE_REMOTE_CONNECTION=OFF \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"

echo "Building depthai-core..."
cmake --build build --parallel $(nproc)

echo "Installing depthai-core to $INSTALL_DIR..."
cmake --install build

echo "==========================================="
echo " depthai-core successfully installed!"
echo " ROS 2 can now build instantly."
echo "==========================================="
