#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Runtime launcher:
#   1. Source ROS2 environment
#   2. Source generated Robonix ROS2 interfaces
#   3. Source n10p lslidar ROS2 workspace overlay
#   4. Configure Python paths
#   5. Start n10p lslidar chassis service
#

set -euo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

cd "$PKG_ROOT"

# --------------------------------------------------
# ROS2 environment
# --------------------------------------------------

ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
else
    echo "[r550_chassis/run] error: ROS2 setup not found:"
    echo "  /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

# --------------------------------------------------
# Robonix generated ROS2 interfaces
# --------------------------------------------------

CODEGEN_SETUP="$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"
if [[ -f "$CODEGEN_SETUP" ]]; then
    echo "[n10p_lslidar/run] sourcing generated ROS2 interfaces"
    # shellcheck disable=SC1091
    set +u
    source "$CODEGEN_SETUP"
    set -u
else
    echo "[n10p_lslidar/run] warning: generated ROS2 interfaces not found:"
    echo "  $CODEGEN_SETUP"
fi
# --------------------------------------------------
# Local ROS2 workspace overlay
# --------------------------------------------------
for cand in \
    "$PKG_ROOT/rbnx-build/ws/install/local_setup.bash" \
    "$PKG_ROOT/rbnx-build/ws/install/setup.bash"
do
    if [[ -f "$cand" ]]; then
        echo "[n10p_lslidar/run] sourcing ROS2 workspace:"
        echo "  $cand"
        # shellcheck disable=SC1091
        set +u
        source "$cand"
        set -u
        break
    fi
done
# --------------------------------------------------
# Python path
# --------------------------------------------------
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG_ROOT:${PYTHONPATH:-}"
else
    export PYTHONPATH="$PKG_ROOT:${PYTHONPATH:-}"
fi

# --------------------------------------------------
# Start service
# --------------------------------------------------

echo "[n10p_lslidar/run] starting n10p_lslidar service"
exec python3 -m n10p_lslidar.main
