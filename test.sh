#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"
GENERATOR="${2:-Ninja}"

CMAKE_ARGS="-DBUILD_TESTS=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5"

echo "==> Configuring with tests enabled..."
cmake -S . -B "${BUILD_DIR}" -G "${GENERATOR}" ${CMAKE_ARGS}

echo "==> Building..."
cmake --build "${BUILD_DIR}"

echo "==> Running tests..."
if [ -f "${BUILD_DIR}/CTestTestfile.cmake" ]; then
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
else
    echo "==> No tests defined yet (tests/ directory is empty)."
fi
