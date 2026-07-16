#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""n10p_lslidar_rbnx — LSLIDAR N10P lidar primitive (capability_id=n10p_lslidar).

Owns `robonix/primitive/lidar/*`. The N10P publishes `/scan` (LaserScan)
via the wheeltec_lidar launch file.

Lifecycle:
    on_init  — parse cfg → spawn lslidar launch → wait for first LaserScan
               → declare ros2 topic_out for primitive/lidar/lidar2d.
    on_shutdown — kill lslidar subprocess.

Config (from manifest's `config:` block, delivered via Driver(CMD_INIT)):
    scan_topic          default "/scan"
    frame_id            default "laser"
    parent_frame        default "base_link"
    extrinsics          optional 6-DoF mount pose of the lidar in
                        parent_frame. Shape: {x, y, z, roll, pitch, yaw}
                        (radians). When present we spawn a
                        static_transform_publisher parent_frame → frame_id
                        so consumers (mapping etc.) see a complete TF
                        tree without needing chassis or soma. Skip when
                        a chassis driver / soma URDF already publishes
                        the same edge.
    sentinel_timeout_s  default 30.0
    launch_package      default "turn_on_wheeltec_robot"
    launch_file         default "wheeltec_lidar.launch.py"
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from robonix_api import Primitive, Ok, Err

logging.basicConfig(
    level=os.environ.get("LSLIDAR_LOG_LEVEL", "INFO"),
    format="[lslidar] %(message)s",
)
log = logging.getLogger("lslidar")

cap = Primitive(id="n10p_lslidar", namespace="robonix/primitive/lidar")


def _pump_output(stream, tag: str) -> None:
    """Forward a child process's merged stdout/stderr into the package
    logger — one unified log stream, no side-car *.log file."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)


_pkg_root: Path = Path(__file__).resolve().parent.parent
_lslidar_proc: subprocess.Popen | None = None
_stp_proc: subprocess.Popen | None = None


# ── lslidar subprocess management ───────────────────────────────────────
def _spawn_lslidar(cfg: dict) -> None:
    global _lslidar_proc

    launch_pkg = str(cfg.get("launch_package", "turn_on_wheeltec_robot"))
    launch_file = str(cfg.get("launch_file", "wheeltec_lidar.launch.py"))

    log.info("spawning lslidar driver: ros2 launch %s %s", launch_pkg, launch_file)
    _lslidar_proc = subprocess.Popen(
        ["ros2", "launch", launch_pkg, launch_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_lslidar_proc.stdout, "lslidar"),
        daemon=True,
    ).start()


def _kill_lslidar() -> None:
    p = _lslidar_proc
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


# ── static_transform_publisher: parent_frame → frame_id ──────────────────
def _spawn_stp(cfg: dict) -> None:
    global _stp_proc
    ext = cfg.get("extrinsics")
    if not ext:
        log.info(
            "no extrinsics in cfg; assuming chassis/soma publishes "
            "parent_frame → frame_id elsewhere"
        )
        return
    parent = str(cfg.get("parent_frame", "base_link"))
    child = str(cfg.get("frame_id", "laser"))
    args = [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", str(float(ext.get("x", 0.0))),
        "--y", str(float(ext.get("y", 0.0))),
        "--z", str(float(ext.get("z", 0.0))),
        "--roll", str(float(ext.get("roll", 0.0))),
        "--pitch", str(float(ext.get("pitch", 0.0))),
        "--yaw", str(float(ext.get("yaw", 0.0))),
        "--frame-id", parent,
        "--child-frame-id", child,
    ]
    log.info("spawning static_transform_publisher %s → %s @ %s",
             parent, child, ext)
    _stp_proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_stp_proc.stdout, "stp"),
        daemon=True,
    ).start()


def _kill_stp() -> None:
    p = _stp_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── sentinel: wait for first LaserScan ───────────────────────────────────
def _wait_for_laserscan(topic: str, timeout_s: float) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import LaserScan
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True

    rclpy.init(args=None)
    node = Node("lslidar_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(LaserScan, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first LaserScan on %s — up to %.1fs", topic, timeout_s)
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
    """REGISTERED → INACTIVE: spawn lslidar, wait for scan, declare topic.

    Self-heal: the N10P may fail to start its data stream on rare occasions.
    Detect it (no LaserScan within sentinel_timeout) and respawn the driver
    up to `retries` times so a remote deploy recovers without manually
    power-cycling the lidar.
    """
    scan_topic = cfg.get("scan_topic", "/scan")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))
    retries = int(cfg.get("retries", 3))

    last_err = ""
    for attempt in range(1, retries + 1):
        try:
            _spawn_lslidar(cfg)
        except Exception as e:  # noqa: BLE001
            return Err(f"spawn lslidar failed: {e}")

        if _wait_for_laserscan(scan_topic, sentinel_timeout):
            if attempt > 1:
                log.info(
                    "lslidar scan stream recovered on attempt %d/%d",
                    attempt, retries,
                )
            break

        last_err = (
            f"no LaserScan on {scan_topic} within "
            f"{sentinel_timeout:.1f}s (attempt {attempt}/{retries})"
        )
        log.warning("%s — respawning lslidar driver", last_err)
        _kill_lslidar()
    else:
        return Err(
            f"{last_err}; lslidar never started its scan stream after "
            f"{retries} respawns — N10P may need a hardware power-cycle."
        )

    # parent_frame → frame_id static TF (no-op when extrinsics absent).
    try:
        _spawn_stp(cfg)
    except Exception as e:  # noqa: BLE001
        _kill_lslidar()
        return Err(f"spawn static_transform_publisher failed: {e}")

    cap.declare_ros2_topic(
        "robonix/primitive/lidar/lidar",
        topic=scan_topic,
        qos="reliable",
    )
    log.info("init complete: lidar=%s", scan_topic)
    return Ok()


@cap.on_shutdown
def shutdown():
    _kill_stp()
    _kill_lslidar()
    return Ok()


if __name__ == "__main__":
    cap.run()

