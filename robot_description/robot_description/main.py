#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""robot_description_driver — Wheeltec mini_tank URDF primitive.

(capability_id=robot_description)

Owns `robonix/primitive/robot_description/driver`.
Wraps the upstream robot model description launch:

    ros2 launch turn_on_wheeltec_robot robot_mode_description_minibot.launch.py mini_tank:=true

This publishes the robot URDF to /robot_description and spawns
robot_state_publisher so that downstream consumers (navigation, rviz,
TF-based services) see a complete TF tree.

The robot_description is NOT bundled into the chassis primitive because:
    - It lives in a different namespace (robonix/primitive/robot_description)
    - The URDF is consumed by many nodes beyond chassis (rviz, nav2, etc.)
    - Follows the "one namespace = one package" invariant

Lifecycle:
    on_init     — spawn minibot model launch → wait for /robot_description
                  (RPC-based, no ROS2 topic exposed).
    on_shutdown — kill subprocess.

Config:
    robot_model         default "mini_tank"
    sentinel_timeout_s  default 30.0
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from robonix_api import Err, Ok, Primitive

logging.basicConfig(
    level=os.environ.get("ROBOT_DESC_LOG_LEVEL", "INFO"),
    format="[robot_desc] %(message)s",
)

log = logging.getLogger("robot_description")

cap = Primitive(
    id="robot_description",
    namespace="robonix/primitive/robot_description",
)

_desc_proc: subprocess.Popen | None = None


def _pump_output(stream, tag: str = "robot_desc") -> None:
    """Forward a child process's stdout/stderr into the package logger."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)


def _spawn_description(cfg: dict) -> None:
    """Spawn robot_mode_description_minibot.launch.py.

    This launch file:
        - loads the URDF for the selected robot model
        - publishes /robot_description
        - starts robot_state_publisher (TF tree from URDF)
    """
    global _desc_proc

    robot_model = str(cfg.get("robot_model", "mini_tank"))

    log.info("spawning robot description: model=%s", robot_model)
    _desc_proc = subprocess.Popen(
        [
            "ros2", "launch",
            "turn_on_wheeltec_robot",
            "robot_mode_description_minibot.launch.py",
            f"{robot_model}:=true",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_desc_proc.stdout, "robot_desc"),
        daemon=True,
    ).start()


def _kill_description() -> None:
    """Terminate the robot description subprocess tree."""
    p = _desc_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def _wait_for_robot_description(timeout_s: float) -> bool:
    """Wait until /robot_description is published."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from std_msgs.msg import String
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True

    rclpy.init(args=None)
    node = Node("robot_desc_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(String, "/robot_description", lambda _m: seen.set(), qos)
    log.info("waiting for /robot_description — up to %.1fs", timeout_s)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if seen.is_set():
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return seen.is_set()


# ── lifecycle handlers ───────────────────────────────────────────────────

@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: spawn model launch, wait for URDF, declare topic."""
    timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    try:
        _spawn_description(cfg)
    except Exception as e:
        return Err(f"spawn robot_description failed: {e}")

    if not _wait_for_robot_description(timeout):
        _kill_description()
        return Err(f"no /robot_description within {timeout:.1f}s")

    log.info("robot_description ready: model=%s", cfg.get("robot_model", "mini_tank"))
    return Ok()


@cap.on_shutdown
def shutdown():
    log.info("stopping robot_description driver")
    _kill_description()
    return Ok()


if __name__ == "__main__":
    cap.run()
