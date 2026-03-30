#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROXSUITE_SRC_DIR="${PKG_DIR}/proxsuite"
PROXSUITE_BUILD_DIR="${PROXSUITE_SRC_DIR}/build"
PROXSUITE_INSTALL_DIR="${PROXSUITE_SRC_DIR}/install"
STAMP_FILE="${PROXSUITE_SRC_DIR}/.installed"

REPO_URL="https://github.com/Simple-Robotics/proxsuite.git"
TRACK_BRANCH="${PROXSUITE_TRACK_BRANCH:-devel}"

if ! command -v git >/dev/null 2>&1; then
  echo "[proxsuite] ERROR: git is required"
  exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
  echo "[proxsuite] ERROR: cmake is required"
  exit 1
fi

echo "[proxsuite] Checking upstream ${TRACK_BRANCH}..."
LATEST_HASH="$(git ls-remote "${REPO_URL}" "refs/heads/${TRACK_BRANCH}" | awk '{print $1}')"

if [[ -z "${LATEST_HASH}" ]]; then
  DEFAULT_REF="$(git ls-remote --symref "${REPO_URL}" HEAD | awk '/^ref:/ {print $2}')"
  DEFAULT_BRANCH="${DEFAULT_REF#refs/heads/}"
  if [[ -n "${DEFAULT_BRANCH}" ]]; then
    echo "[proxsuite] WARNING: branch '${TRACK_BRANCH}' not found. Falling back to default branch '${DEFAULT_BRANCH}'."
    TRACK_BRANCH="${DEFAULT_BRANCH}"
    LATEST_HASH="$(git ls-remote "${REPO_URL}" "refs/heads/${TRACK_BRANCH}" | awk '{print $1}')"
  fi
fi

# Fallback if ls-remote fails (offline): keep existing install if present
if [[ -z "${LATEST_HASH}" ]]; then
  if [[ -f "${STAMP_FILE}" ]]; then
    echo "[proxsuite] WARNING: could not reach GitHub. Reusing installed commit $(cat "${STAMP_FILE}")"
    exit 0
  fi
  echo "[proxsuite] ERROR: could not resolve latest commit and no local install exists"
  exit 1
fi

echo "[proxsuite] Target commit: ${LATEST_HASH} (branch: ${TRACK_BRANCH})"

CURRENT_HASH=""
if [[ -f "${STAMP_FILE}" ]]; then
  CURRENT_HASH="$(cat "${STAMP_FILE}")"
fi

if [[ "${CURRENT_HASH}" == "${LATEST_HASH}" && -d "${PROXSUITE_INSTALL_DIR}" ]]; then
  echo "[proxsuite] Already up to date (${CURRENT_HASH})"
  exit 0
fi

if [[ ! -d "${PROXSUITE_SRC_DIR}/.git" ]]; then
  echo "[proxsuite] Cloning repository..."
  rm -rf "${PROXSUITE_SRC_DIR}"
  git clone --recursive --branch "${TRACK_BRANCH}" "${REPO_URL}" "${PROXSUITE_SRC_DIR}"
fi

echo "[proxsuite] Fetching latest changes from ${TRACK_BRANCH}..."
git -C "${PROXSUITE_SRC_DIR}" fetch origin "${TRACK_BRANCH}" --tags --prune

echo "[proxsuite] Checking out and pulling ${TRACK_BRANCH}"
git -C "${PROXSUITE_SRC_DIR}" checkout "${TRACK_BRANCH}"
git -C "${PROXSUITE_SRC_DIR}" pull --ff-only origin "${TRACK_BRANCH}"

LOCAL_HASH="$(git -C "${PROXSUITE_SRC_DIR}" rev-parse HEAD)"
if [[ "${LOCAL_HASH}" != "${LATEST_HASH}" ]]; then
  echo "[proxsuite] WARNING: local hash (${LOCAL_HASH}) differs from remote hash (${LATEST_HASH})."
fi

git -C "${PROXSUITE_SRC_DIR}" submodule update --init --recursive

echo "[proxsuite] Configuring..."
rm -rf "${PROXSUITE_BUILD_DIR}" "${PROXSUITE_INSTALL_DIR}"
cmake -S "${PROXSUITE_SRC_DIR}" -B "${PROXSUITE_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PROXSUITE_INSTALL_DIR}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF

echo "[proxsuite] Building..."
cmake --build "${PROXSUITE_BUILD_DIR}" -j"$(nproc)"

echo "[proxsuite] Installing..."
if ! cmake --install "${PROXSUITE_BUILD_DIR}" 2>&1 | sed '/^Error: could not load cache$/d'; then
  echo "[proxsuite] ERROR: install step failed"
  exit 1
fi

if [[ ! -d "${PROXSUITE_INSTALL_DIR}/include" ]]; then
  echo "[proxsuite] ERROR: install/include not found"
  exit 1
fi

echo "${LOCAL_HASH}" > "${STAMP_FILE}"
echo "[proxsuite] Installation successful at commit ${LOCAL_HASH}"
