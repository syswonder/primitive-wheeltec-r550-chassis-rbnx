#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""astra_s_camera_rbnx — Orbbec Astra S RGB-D camera primitive
(capability_id=astra_s_camera).

Owns `robonix/primitive/camera/*`. Directly launches the astra_camera
driver via ros2 launch astra_camera astra.launch.xml (the same
underlying include that wheeltec_camera.launch.py uses).

Capability surface:
  primitive/camera/driver         rpc gRPC (lifecycle)
  primitive/camera/rgb            topic_out ROS2 (continuous, raw)
  primitive/camera/depth          topic_out ROS2 (continuous, raw)
  primitive/camera/intrinsics     topic_out ROS2 (bound directly to camera_info topic)
  primitive/camera/snapshot       rpc MCP (one-shot RGB JPEG — VLM-facing)
  primitive/camera/depth_snapshot rpc MCP (one-shot depth as 8-bit JPEG)
  primitive/camera/extrinsics     (declared in package_manifest only; not implemented)

Lifecycle:
    on_init      — ros2 launch astra_camera astra.launch.xml
                   → subscribe rgb + depth →
                   wait for first RGB frame →
                   declare rgb/depth/intrinsics topic_out
                   + snapshot + depth_snapshot MCP.
    on_shutdown  — kill camera subprocess.

Config (from manifest):
    color_topic          default "/camera/color/image_raw"
    depth_topic          default "/camera/depth/image_raw"
    intrinsics_topic     default "/camera/color/camera_info"
    cam_frame            default "camera_link"
    launch_package       default "astra_camera"
    launch_file          default "astra.launch.xml"
    sentinel_timeout_s   default 60.0
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from io import BytesIO
from pathlib import Path

import numpy as np

from robonix_api import Primitive, Ok, Err

logging.basicConfig(
    level=os.environ.get("ASTRA_CAMERA_LOG_LEVEL", "INFO"),
    format="[astra_camera] %(message)s",
)
log = logging.getLogger("astra_camera")

cap = Primitive(id="astra_s_camera", namespace="robonix/primitive/camera")

_pkg_root: Path = Path(__file__).resolve().parent.parent
_camera_proc: subprocess.Popen | None = None

# ── snapshot state ───────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_latest_rgb_jpeg: bytes | None = None
_latest_depth_jpeg: bytes | None = None
_rgb_frame_id: str = "camera_link"
_depth_frame_id: str = "camera_link"
_rgb_received = threading.Event()


# ── camera subprocess management ──────────────────────────────────────────
def _spawn_camera(cfg: dict) -> None:
    """Launch ros2 launch <launch_package> <launch_file>.

    Before launching, aggressively clean up any leftover camera processes
    that might still hold the USB device from a previous run — this is the
    most common failure mode: a prior shutdown didn't fully tear down the
    node tree, so the Astra S stays "Resource busy".
    """
    global _camera_proc

    launch_pkg = str(cfg.get("launch_package", "astra_camera"))
    launch_file = str(cfg.get("launch_file", "astra.launch.xml"))

    # ── pre-launch cleanup: nuke any leftover camera processes ──────────
    for target in ("astra_camera_node", "ros2 launch astra_camera"):
        result = subprocess.run(
            ["pkill", "-f", target],
            check=False,
        )
        log.debug("pre-launch pkill -f %s → rc=%d", target, result.returncode)

    # Brief settle so the kernel can release the USB interface.
    time.sleep(0.5)

    log_path = _pkg_root / "rbnx-build" / "data" / "astra_camera.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning camera driver: ros2 launch %s %s → %s",
             launch_pkg, launch_file, log_path)
    _camera_proc = subprocess.Popen(
        ["ros2", "launch", launch_pkg, launch_file],
        stdout=log_fh, stderr=log_fh, start_new_session=True,
    )


def _kill_camera() -> None:
    """Tear down the camera subprocess tree, including orphaned children.

    ros2 launch can spawn the actual astra_camera_node in a different
    process group than the launch parent, so killpg alone is not enough —
    we also use pkill as a safety net to catch any stragglers that would
    hold the USB device and cause "Resource busy" on the next start.
    """
    global _camera_proc

    p = _camera_proc
    if p is None:
        return

    # 1) Gentle shutdown: ROS nodes handle SIGINT cleanly (close USB, etc.).
    if p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

    # 2) Wait a few seconds for graceful teardown.
    try:
        p.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        # 3) Force-kill anything still alive in the process group.
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            log.warning("camera subprocess did not die after SIGKILL")

    # 4) Belt-and-suspenders: pkill any orphaned camera nodes that escaped
    #    the process group (classic ros2 launch behaviour).
    for target in ("astra_camera_node", "ros2 launch astra_camera"):
        subprocess.run(
            ["pkill", "-f", target],
            check=False,
        )

    _camera_proc = None


# ── image conversion ─────────────────────────────────────────────────────────
def _ros_image_to_jpeg(msg) -> bytes:
    """Encode a sensor_msgs/Image into JPEG bytes.
    Supports: rgb8, bgr8, rgba8, bgra8, mono8, 16uc1, 32fc1."""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
    elif enc == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif enc == "bgra8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
    elif enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "16uc1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = (raw / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else np.zeros((h, w), np.uint8)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        valid = np.isfinite(raw)
        if valid.any():
            mn, mx = raw[valid].min(), raw[valid].max()
            norm = np.where(valid, (raw - mn) / max(mx - mn, 1e-6) * 255, 0).astype(np.uint8)
        else:
            norm = np.zeros((h, w), np.uint8)
        arr = np.stack([norm, norm, norm], axis=-1)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.fromarray(np.ascontiguousarray(arr)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _on_rgb(msg) -> None:
    global _latest_rgb_jpeg, _rgb_frame_id
    _rgb_received.set()
    try:
        jpg = _ros_image_to_jpeg(msg)
        with _state_lock:
            _latest_rgb_jpeg = jpg
            if msg.header.frame_id:
                _rgb_frame_id = msg.header.frame_id
    except Exception as e:  # noqa: BLE001
        log.warning("RGB conversion error: %s", e)


def _on_depth(msg) -> None:
    global _latest_depth_jpeg, _depth_frame_id
    try:
        jpg = _ros_image_to_jpeg(msg)
        with _state_lock:
            _latest_depth_jpeg = jpg
            if msg.header.frame_id:
                _depth_frame_id = msg.header.frame_id
    except Exception as e:  # noqa: BLE001
        log.warning("depth conversion error: %s", e)


# ── MCP snapshot tools (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def _now_header(frame_id: str) -> std_msgs_mcp.Header:
    now = time.time()
    sec = int(now)
    ns = int((now % 1) * 1e9) % 1_000_000_000
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=ns),
        frame_id=frame_id,
    )


def _jpeg_to_image_mcp(jpg: bytes, frame_id: str) -> Image:
    from PIL import Image as PILImage
    im = PILImage.open(BytesIO(jpg))
    w, h = im.size
    return Image(
        header=_now_header(frame_id),
        height=h, width=w,
        encoding="jpeg",
        is_bigendian=0,
        step=len(jpg),
        data=jpg,
    )


def _empty_image_error(reason: str) -> Image:
    """Return a tiny black 1x1 JPEG when we can't deliver a frame."""
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.new("RGB", (1, 1), (0, 0, 0)).save(buf, format="JPEG")
    return _jpeg_to_image_mcp(buf.getvalue(), f"error:{reason}")


@cap.mcp("robonix/primitive/camera/snapshot")
def snapshot(msg: Empty) -> Image:
    """PRIMARY perception tool. Returns the current RGB frame as a
    JPEG-encoded sensor_msgs/Image (encoding='jpeg', data=JPEG bytes)."""
    with _state_lock:
        data = _latest_rgb_jpeg
        frame_id = _rgb_frame_id
    if data is None:
        return _empty_image_error("no RGB frame received yet")
    return _jpeg_to_image_mcp(data, frame_id)


@cap.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(msg: Empty) -> Image:
    """Depth snapshot as 8-bit JPEG (normalized for visualization).
    Returns sensor_msgs/Image with encoding='jpeg'. For actual metric
    depth, subscribe to robonix/primitive/camera/depth (16UC1)."""
    with _state_lock:
        data = _latest_depth_jpeg
        frame_id = _depth_frame_id
    if data is None:
        return _empty_image_error("no depth frame received yet")
    return _jpeg_to_image_mcp(data, frame_id)


# ── lifecycle ────────────────────────────────────────────────────────────────
@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: spawn camera, subscribe RGB+depth, declare."""
    color_topic = cfg.get("color_topic", "/camera/color/image_raw")
    depth_topic = cfg.get("depth_topic", "/camera/depth/image_raw")
    intrinsics_topic = cfg.get("intrinsics_topic", "/camera/color/camera_info")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 60.0))
    _rgb_received.clear()

    try:
        _spawn_camera(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn camera failed: {e}")

    # Subscribe RGB + depth via robonix_api (declare=False — we declare
    # the ros2 topic_out interfaces explicitly below, after sentinel passes).
    cap.create_subscription(
        "robonix/primitive/camera/rgb",
        topic=color_topic, msg_type="Image",
        callback=_on_rgb, qos="reliable", declare=False,
    )
    cap.create_subscription(
        "robonix/primitive/camera/depth",
        topic=depth_topic, msg_type="Image",
        callback=_on_depth, qos="reliable", declare=False,
    )

    # Gate INIT on the permanent RGB subscription. USB cameras can lag on cold
    # boot, but no temporary ROS subscription should be created or destroyed
    # while the process-wide executor is spinning.
    if not _rgb_received.wait(timeout=sentinel_timeout):
        _kill_camera()
        return Err(f"no Image on {color_topic} within {sentinel_timeout:.1f}s")

    # Declare topic_out capabilities on Atlas. intrinsics binds directly to
    # the driver's CameraInfo topic — the Astra S publishes it natively.
    cap.declare_ros2_topic(
        "robonix/primitive/camera/rgb",
        topic=color_topic, qos="reliable",
    )
    cap.declare_ros2_topic(
        "robonix/primitive/camera/depth",
        topic=depth_topic, qos="reliable",
    )
    cap.declare_ros2_topic(
        "robonix/primitive/camera/intrinsics",
        topic=intrinsics_topic, qos="reliable",
    )

    log.info("init complete: rgb=%s depth=%s intrinsics=%s "
             "+ snapshot/depth_snapshot MCP exposed",
             color_topic, depth_topic, intrinsics_topic)
    return Ok()


@cap.on_shutdown
def shutdown():
    _kill_camera()
    return Ok()


if __name__ == "__main__":
    cap.run()
