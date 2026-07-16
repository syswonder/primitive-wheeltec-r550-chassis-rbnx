#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

rbnx codegen -p "$PKG" --ros2

set +u
source /opt/ros/humble/setup.bash
set -u
( cd "$PKG/rbnx-build/codegen/ros2_idl" && colcon build )

echo "[astra_s_camera] build done"
