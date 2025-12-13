#!/usr/bin/env bash
set -e

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MUJOCO_DIR="${PKG_DIR}/mujoco"
MUJOCO_VERSION="3.4.0"
STAMP_FILE="${MUJOCO_DIR}/.installed"

ARCHIVE="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${ARCHIVE}"

# Skip if already installed
if [ -f "${STAMP_FILE}" ]; then
    echo "[MuJoCo] Already installed → skipping."
    exit 0
fi

echo "[MuJoCo] Installing into ${MUJOCO_DIR}"

# Download & extract
wget -q "${URL}" -O "/tmp/${ARCHIVE}"
rm -rf "${MUJOCO_DIR}"
mkdir -p "${MUJOCO_DIR}"
tar -xzf "/tmp/${ARCHIVE}" -C "${MUJOCO_DIR}" --strip-components=1
rm "/tmp/${ARCHIVE}"

# Test loading
python3 - << EOF
import ctypes, os
try:
    ctypes.CDLL(os.path.join("${MUJOCO_DIR}", "lib/libmujoco.so"))
    print("[MuJoCo] Load test OK")
except Exception as e:
    print("[MuJoCo] Load test FAILED:", e)
    exit(1)
EOF

# Mark install complete
touch "${STAMP_FILE}"

echo "[MuJoCo] Install complete."
