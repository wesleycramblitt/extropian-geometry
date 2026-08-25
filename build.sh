#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"
GENERATOR="${2:-Ninja}"

# CMake 4.x requires explicit policy minimum for older FetchContent deps
CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"

echo "==> Configuring (${GENERATOR})..."
cmake -S . -B "${BUILD_DIR}" -G "${GENERATOR}" ${CMAKE_ARGS}

echo "==> Building..."
cmake --build "${BUILD_DIR}"

echo "==> Build complete: ${BUILD_DIR}/libexd-geometry.a"
