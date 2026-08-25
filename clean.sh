#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"

echo "==> Cleaning ${BUILD_DIR}..."
rm -rf "${BUILD_DIR}"
echo "==> Clean."
