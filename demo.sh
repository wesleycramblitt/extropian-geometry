#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

BUILD_DIR="${1:-build}"
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Prefer local sibling checkouts of the exd dependencies when present (faster,
# uses local fixes); fall back to FetchContent (GitHub main) otherwise.
EXTRA_ARGS=()
for entry in \
    "EXD_CORE_DIR extropian-core" \
    "EXD_APP_DIR extropian-app" \
    "EXD_RENDER_DIR extropian-render"
do
    var="${entry%% *}"
    dir="${ROOT_DIR}/../${entry##* }"
    if [ -d "${dir}" ]; then
        EXTRA_ARGS+=("-D${var}=${dir}")
        echo "==> Using local ${entry##* } at ${dir}"
    fi
done

echo "==> Configuring (demo mode)..."
cmake -B "${BUILD_DIR}" -G Ninja -DBUILD_DEMO=ON -DBUILD_TESTS=OFF \
    -DEXD_ASSETS_DIR="${ROOT_DIR}/../extropian-assets" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "${EXTRA_ARGS[@]}"

echo "==> Building demo..."
cmake --build "${BUILD_DIR}" --target extropian-geometry-gallery -j "$(nproc)"

echo "==> Running..."
cd "${BUILD_DIR}" && ./demo/extropian-geometry-gallery
