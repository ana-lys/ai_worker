#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROXSUITE_SRC_DIR="${PKG_DIR}/proxsuite"
PROXSUITE_BUILD_DIR="${PROXSUITE_SRC_DIR}/build"
PROXSUITE_INSTALL_DIR="${PROXSUITE_SRC_DIR}/install"
STAMP_FILE="${PROXSUITE_SRC_DIR}/.installed"

REPO_URL="https://github.com/Simple-Robotics/proxsuite.git"
TRACK_BRANCH="devel"

command -v git >/dev/null 2>&1 || { echo "[proxsuite] ERROR: git is required"; exit 1; }
command -v cmake >/dev/null 2>&1 || { echo "[proxsuite] ERROR: cmake is required"; exit 1; }

echo "[proxsuite] Using branch ${TRACK_BRANCH}"

if [[ ! -d "${PROXSUITE_SRC_DIR}/.git" ]]; then
  echo "[proxsuite] Cloning ${REPO_URL} (${TRACK_BRANCH})"
  rm -rf "${PROXSUITE_SRC_DIR}"
  git clone --recursive --branch "${TRACK_BRANCH}" "${REPO_URL}" "${PROXSUITE_SRC_DIR}"
else
  echo "[proxsuite] Updating existing checkout"
  git -C "${PROXSUITE_SRC_DIR}" fetch origin "${TRACK_BRANCH}" --tags --prune
  git -C "${PROXSUITE_SRC_DIR}" checkout "${TRACK_BRANCH}"
  git -C "${PROXSUITE_SRC_DIR}" pull --ff-only origin "${TRACK_BRANCH}"
  git -C "${PROXSUITE_SRC_DIR}" submodule update --init --recursive
fi

LOCAL_HASH="$(git -C "${PROXSUITE_SRC_DIR}" rev-parse HEAD)"
echo "[proxsuite] Building commit ${LOCAL_HASH}"

rm -rf "${PROXSUITE_BUILD_DIR}" "${PROXSUITE_INSTALL_DIR}"
cmake -S "${PROXSUITE_SRC_DIR}" -B "${PROXSUITE_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PROXSUITE_INSTALL_DIR}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF

cmake --build "${PROXSUITE_BUILD_DIR}" -j"$(nproc)"
cmake --install "${PROXSUITE_BUILD_DIR}" 2>&1 | sed '/^Error: could not load cache$/d'

echo "${LOCAL_HASH}" > "${STAMP_FILE}"
echo "[proxsuite] Installed to ${PROXSUITE_INSTALL_DIR} (commit ${LOCAL_HASH})"
