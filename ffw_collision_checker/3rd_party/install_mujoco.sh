#!/usr/bin/env bash
set -e

# 1. Setup Directories
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MUJOCO_DIR="${PKG_DIR}/mujoco"
STAMP_FILE="${MUJOCO_DIR}/.installed"

# 2. Detect Architecture
ARCH=$(uname -m)
case "${ARCH}" in
    x86_64)  MUJOCO_ARCH="linux-x86_64" ;;
    aarch64) MUJOCO_ARCH="linux-aarch64" ;;
    *) echo "Error: Unsupported architecture ${ARCH}"; exit 1 ;;
esac

# 3. Target fixed version
TAG_NAME="3.4.0"
LATEST_VER="3.4.0"
echo "[MuJoCo] Target version: ${LATEST_VER} (Tag: ${TAG_NAME})"

# 4. Version Check / Skip Logic
if [ -f "${STAMP_FILE}" ]; then
    CURRENT_VER=$(cat "${STAMP_FILE}")
    if [ "$CURRENT_VER" == "$LATEST_VER" ]; then
        echo "[MuJoCo] Version ${CURRENT_VER} is already up to date."
        exit 0
    else
        echo "[MuJoCo] Update found! ${CURRENT_VER} -> ${LATEST_VER}. Updating..."
    fi
fi

# 5. Download & Extract
# URL format: .../download/{TAG}/{FILENAME}
ARCHIVE="mujoco-${LATEST_VER}-${MUJOCO_ARCH}.tar.gz"
URL="https://github.com/google-deepmind/mujoco/releases/download/${TAG_NAME}/${ARCHIVE}"

echo "[MuJoCo] Downloading from: ${URL}"

rm -rf "${MUJOCO_DIR}"
mkdir -p "${MUJOCO_DIR}"

TMP_ARCHIVE="$(mktemp /tmp/mujoco.XXXXXX.tar.gz)"
cleanup() { rm -f "${TMP_ARCHIVE}"; }
trap cleanup EXIT

if ! curl -L --fail --retry 3 --retry-delay 2 -o "${TMP_ARCHIVE}" "${URL}"; then
    echo "[MuJoCo] ERROR: download failed for ${URL}"
    exit 1
fi

if ! tar -xz -C "${MUJOCO_DIR}" --strip-components=1 -f "${TMP_ARCHIVE}"; then
    echo "[MuJoCo] ERROR: extraction failed for ${TMP_ARCHIVE}"
    exit 1
fi

# 6. Load Test
echo "[MuJoCo] Running load test..."
export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH}"

python3 - << EOF
import ctypes, os, sys
lib_path = os.path.join("${MUJOCO_DIR}", "lib/libmujoco.so")
try:
    ctypes.CDLL(lib_path)
    print(f"[MuJoCo] Load test OK")
except Exception as e:
    print(f"[MuJoCo] Load test FAILED: {e}")
    sys.exit(1)
EOF

# 7. Finalize
echo "${LATEST_VER}" > "${STAMP_FILE}"
echo "[MuJoCo] Installation of ${LATEST_VER} successful."
