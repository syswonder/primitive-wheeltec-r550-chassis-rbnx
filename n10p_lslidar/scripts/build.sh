#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

rbnx codegen -p "$PKG" --ros2

source /opt/ros/humble/setup.bash
( cd "$PKG/rbnx-build/codegen/ros2_idl" && colcon build )

echo "[n10p_lslidar] build done"
