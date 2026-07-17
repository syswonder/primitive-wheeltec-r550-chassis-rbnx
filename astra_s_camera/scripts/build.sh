#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build phase: rbnx codegen + ROS 2 IDL colcon build.
#
# Unlike orbbec_camera_rbnx, we depend on `turn_on_wheeltec_robot` which
# is a source-built ROS 2 package. Its overlay workspace lives under
# rbnx-build/ws/ and must already be colcon-built before start.sh runs
# (typically done by the deploy orchestrator or a pre-build step).
#
# Codegen generates three output trees under rbnx-build/codegen/:
#   - proto_gen/          gRPC Python stubs (atlas_pb2 + lifecycle_pb2)
#   - robonix_mcp_types/  MCP dataclasses (builtin_interfaces_mcp,
#                         std_msgs_mcp, sensor_msgs_mcp, …)
#   - ros2_idl/           ROS 2 canonical message overlay (colcon-built below)
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[astra_s_camera/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/data

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --ros2 --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[astra_s_camera/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# colcon build the ROS 2 IDL overlay so start.sh can source
# rbnx-build/codegen/ros2_idl/install/setup.bash
set +u
source /opt/ros/humble/setup.bash
set -u
( cd "$PKG/rbnx-build/codegen/ros2_idl" && colcon build )

touch "$PKG/rbnx-build/.rbnx-built"
echo "[astra_s_camera/build] done."
