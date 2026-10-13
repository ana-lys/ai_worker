#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROXSUITE_SRC_DIR="${PKG_DIR}/proxsuite"
PROXSUITE_BUILD_DIR="${PROXSUITE_SRC_DIR}/build"
PROXSUITE_INSTALL_DIR="${PROXSUITE_SRC_DIR}/install"
STAMP_FILE="${PROXSUITE_SRC_DIR}/.installed"

REPO_URL="https://github.com/Simple-Robotics/proxsuite.git"
# Pinned: tracking devel rebuilt proxsuite on every configure and broke the
# build when upstream moved to jrl-cmakemodules v2 (PR #437 "jrl-next" made
# cereal + nanobind hard requirements). 5c30aad (v0.7.3+35, 2026-09-14) is the
# last commit this package built and ran with. Bump deliberately.
PIN_COMMIT="5c30aad1e627d471198826f5e0b12f90be050ee6"

command -v git >/dev/null 2>&1 || { echo "[proxsuite] ERROR: git is required"; exit 1; }
command -v cmake >/dev/null 2>&1 || { echo "[proxsuite] ERROR: cmake is required"; exit 1; }

# Already built at the pinned commit: nothing to do.
if [[ -f "${STAMP_FILE}" && "$(cat "${STAMP_FILE}")" == "${PIN_COMMIT}" && -d "${PROXSUITE_INSTALL_DIR}" ]]; then
  echo "[proxsuite] ${PIN_COMMIT} already installed"
  exit 0
fi

echo "[proxsuite] Using pinned commit ${PIN_COMMIT}"
if [[ ! -d "${PROXSUITE_SRC_DIR}/.git" ]]; then
  echo "[proxsuite] Cloning ${REPO_URL}"
  rm -rf "${PROXSUITE_SRC_DIR}"
  git clone "${REPO_URL}" "${PROXSUITE_SRC_DIR}"
fi
if ! git -C "${PROXSUITE_SRC_DIR}" cat-file -e "${PIN_COMMIT}^{commit}" 2>/dev/null; then
  git -C "${PROXSUITE_SRC_DIR}" fetch origin --tags --prune
fi
git -C "${PROXSUITE_SRC_DIR}" checkout --detach "${PIN_COMMIT}"
git -C "${PROXSUITE_SRC_DIR}" submodule update --init --recursive

LOCAL_HASH="$(git -C "${PROXSUITE_SRC_DIR}" rev-parse HEAD)"
echo "[proxsuite] Building commit ${LOCAL_HASH}"

rm -rf "${PROXSUITE_BUILD_DIR}" "${PROXSUITE_INSTALL_DIR}"
cmake -S "${PROXSUITE_SRC_DIR}" -B "${PROXSUITE_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PROXSUITE_INSTALL_DIR}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DPROXSUITE_BUILD_WITH_SERIALIZATION=OFF

cmake --build "${PROXSUITE_BUILD_DIR}" -j"$(nproc)"
cmake --install "${PROXSUITE_BUILD_DIR}" 2>&1 | sed '/^Error: could not load cache$/d'

echo "${LOCAL_HASH}" > "${STAMP_FILE}"
echo "[proxsuite] Installed to ${PROXSUITE_INSTALL_DIR} (commit ${LOCAL_HASH})"
