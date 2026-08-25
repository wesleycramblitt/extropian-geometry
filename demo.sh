#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

BUILD_DIR="${1:-build}"

echo "==> Configuring (demo mode)..."
cmake -B "${BUILD_DIR}" -G Ninja -DBUILD_DEMO=ON -DBUILD_TESTS=OFF \
    -DEXD_ASSETS_DIR="$(dirname "$0")/../extropian-assets" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5

echo "==> Building demo..."
cmake --build "${BUILD_DIR}" --target extropian-geometry-shapes -j "$(nproc)"

echo "==> Running..."
cd "${BUILD_DIR}" && ./demo/extropian-geometry-shapes
