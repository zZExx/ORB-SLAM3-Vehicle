#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CORE_DIR="${ORB_SLAM3_CORE_DIR:-${SCRIPT_DIR}/../ORB_SLAM3}"
CORE_DIR="$(cd "${CORE_DIR}" && pwd)"
echo "Building ORB_SLAM3 core in: ${CORE_DIR}"
cd "${CORE_DIR}"
bash ./build.sh
