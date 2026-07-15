#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""r550_chassis — Wheeltec R550 chassis primitive (mini_tank).

(capability_id=r550_chassis)

Owns `robonix/primitive/chassis` and `robonix/primitive/imu`.
Wraps the upstream Wheeltec ROS2 base driver and all chassis-sidecar nodes:

    ros2 launch turn_on_wheeltec_robot base_serial.launch.py
    + static_transform_publisher  base_footprint → base_link
    + static_transform_publisher  base_footprint → gyro_link
    + joint_state_publisher       (all joints fixed — mini_tank)
    + imu_filter_madgwick         (raw → filtered IMU orientation)
    + wheeltec_ekf                (fused odom + imu → /odom_combined)

The primitive does NOT directly access the serial port.
The Wheeltec ROS2 driver owns:
    - /dev/wheeltec_controller
    - serial protocol
    - MCU communication
    - motor control

Lifecycle:
    on_init (chassis):
        - spawn wheeltec base driver
        - spawn base→link & base→gyro static TFs
        - spawn joint_state_publisher (fixed joints only)
        - spawn imu_filter_madgwick
        - spawn wheeltec_ekf (fuses /odom + /imu_data → /odom_combined)
        - wait for first /odom_combined message
        - declare chassis/odom (→ /odom_combined) and chassis/twist_in

    on_init (imu):
        - wait for first /imu_data message (filtered IMU)
        - declare imu/imu

    on_shutdown:
        - terminate all subprocesses

Config:
    odom_topic_name         default "/odom_combined"    (wheeltec_ekf fused output)
    cmd_vel_topic           default "/cmd_vel"           (chassis twist command)
    imu_topic               default "/imu_data"          (imu_filter_madgwick output)
    base_to_link_z          default 0.03715              (mini_tank z offset)
    enable_imu_filter       default true
    enable_joint_state_pub  default true
    enable_ekf              default true
    sentinel_timeout_s      default 30.0
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
    level=os.environ.get("R550_CHASSIS_LOG_LEVEL", "INFO"),
    format="[r550] %(message)s",
)

log = logging.getLogger("r550_chassis")

provider = Primitive(id="r550_chassis", namespace="robonix/primitive/chassis")
imu_provider = Primitive(id="r550_imu", namespace="robonix/primitive/imu")

# ── subprocess handles ───────────────────────────────────────────────────
_r550_proc: subprocess.Popen | None = None
_base_to_link_proc: subprocess.Popen | None = None
_base_to_gyro_proc: subprocess.Popen | None = None
_joint_state_proc: subprocess.Popen | None = None
_imu_filter_proc: subprocess.Popen | None = None
_ekf_proc: subprocess.Popen | None = None


def _pump_output(stream, tag: str = "wheeltec_base") -> None:
    """Forward a child process's stdout/stderr into the package logger."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)


# ── spawn / kill helpers ──────────────────────────────────────────────────

def _spawn_wheeltec(cfg: dict) -> None:
    """Spawn Wheeltec R550 ROS2 base driver (base_serial.launch.py)."""
    global _r550_proc
    args = [
        "ros2", "launch",
        "turn_on_wheeltec_robot",
        "base_serial.launch.py",
    ]
    log.info("spawning wheeltec R550 base driver")
    log.debug("launch command: %s", " ".join(args))
    _r550_proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_r550_proc.stdout, "base_serial"),
        daemon=True,
    ).start()


def _spawn_base_to_link(cfg: dict) -> None:
    """Spawn static_transform_publisher: base_footprint → base_link.

    The z offset is vehicle-model dependent.  For mini_tank it is 0.03715.
    See the full table in wheeltec_robot.launch.py for other models.
    """
    global _base_to_link_proc
    z = str(float(cfg.get("base_to_link_z", 0.03715)))
    args = [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", "0", "--y", "0", "--z", z,
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "base_footprint",
        "--child-frame-id", "base_link",
    ]
    log.info("spawning static_transform_publisher base_footprint → base_link (z=%s)", z)
    _base_to_link_proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_base_to_link_proc.stdout, "base_to_link"),
        daemon=True,
    ).start()


def _spawn_base_to_gyro(cfg: dict) -> None:
    """Spawn static_transform_publisher: base_footprint → gyro_link.

    The IMU is rigidly mounted inside the chassis — all zeros for mini_tank.
    """
    global _base_to_gyro_proc
    args = [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "base_footprint",
        "--child-frame-id", "gyro_link",
    ]
    log.info("spawning static_transform_publisher base_footprint → gyro_link")
    _base_to_gyro_proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_base_to_gyro_proc.stdout, "base_to_gyro"),
        daemon=True,
    ).start()


def _spawn_joint_state_publisher(cfg: dict) -> None:
    """Spawn joint_state_publisher.

    NOTE: The mini_tank chassis has NO movable joints — all joints in the
    URDF are FIXED.  This node therefore publishes a static joint_state
    message (all zeros) so that robot_state_publisher can build the full TF
    tree.  If a future chassis model adds actuated joints, this node will
    need additional configuration.
    """
    global _joint_state_proc
    if not cfg.get("enable_joint_state_pub", True):
        log.info("joint_state_publisher disabled via config")
        return
    log.info("spawning joint_state_publisher (all joints fixed)")
    _joint_state_proc = subprocess.Popen(
        ["ros2", "run", "joint_state_publisher", "joint_state_publisher"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_joint_state_proc.stdout, "joint_state"),
        daemon=True,
    ).start()


def _spawn_imu_filter(cfg: dict) -> None:
    """Spawn imu_filter_madgwick to produce filtered IMU orientation.

    Consumes /imu/data_raw (or cfg.imu_topic) and publishes /imu/data
    (or cfg.imu_topic_filtered).  The Madgwick filter fuses gyro +
    accelerometer to estimate orientation; this is sensor-level signal
    processing, NOT a SLAM/localization filter.
    """
    global _imu_filter_proc
    if not cfg.get("enable_imu_filter", True):
        log.info("imu_filter_madgwick disabled via config")
        return

    # Build a temporary parameter file so we can override topic names.
    # The default imu_filter_madgwick params live in
    #   <turn_on_wheeltec_robot>/config/imu.yaml
    # We pass it directly — the node resolves the share directory internally.
    imu_config_path = cfg.get("imu_filter_config")
    if imu_config_path:
        log.info("spawning imu_filter_madgwick with config: %s", imu_config_path)
        _imu_filter_proc = subprocess.Popen(
            [
                "ros2", "run", "imu_filter_madgwick", "imu_filter_madgwick_node",
                "--ros-args", "-p", f"config_file:={imu_config_path}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    else:
        log.info("spawning imu_filter_madgwick (default params)")
        _imu_filter_proc = subprocess.Popen(
            ["ros2", "run", "imu_filter_madgwick", "imu_filter_madgwick_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    threading.Thread(
        target=_pump_output,
        args=(_imu_filter_proc.stdout, "imu_filter"),
        daemon=True,
    ).start()


def _spawn_ekf(cfg: dict) -> None:
    """Spawn wheeltec_ekf (robot_localization EKF).

    Fuses /odom (wheel odometry) + /imu_data (filtered IMU) → /odom_combined.
    This is the final, drift-corrected odometry that downstream consumers
    (nav2, mapping) should use.
    """
    global _ekf_proc
    if not cfg.get("enable_ekf", True):
        log.info("wheeltec_ekf disabled via config")
        return
    log.info("spawning wheeltec_ekf (carto_slam=false)")
    _ekf_proc = subprocess.Popen(
        [
            "ros2", "launch",
            "turn_on_wheeltec_robot",
            "wheeltec_ekf.launch.py",
            "carto_slam:=false",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output,
        args=(_ekf_proc.stdout, "ekf"),
        daemon=True,
    ).start()


# ── kill helpers ──────────────────────────────────────────────────────────

def _kill_proc(p: subprocess.Popen | None, name: str, timeout_s: float = 5.0) -> None:
    """Gracefully terminate a subprocess tree, then force-kill if needed."""
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        log.warning("%s did not exit after SIGTERM; sending SIGKILL", name)
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def _kill_all() -> None:
    """Terminate all subprocesses in reverse dependency order."""
    _kill_proc(_ekf_proc, "ekf", timeout_s=3.0)
    _kill_proc(_imu_filter_proc, "imu_filter", timeout_s=3.0)
    _kill_proc(_joint_state_proc, "joint_state_publisher", timeout_s=3.0)
    _kill_proc(_base_to_gyro_proc, "base_to_gyro", timeout_s=2.0)
    _kill_proc(_base_to_link_proc, "base_to_link", timeout_s=2.0)
    _kill_proc(_r550_proc, "base_serial", timeout_s=5.0)


# ── sentinels ─────────────────────────────────────────────────────────────

def _wait_for_imu(topic: str, timeout_s: float) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Imu
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("wheeltec_r550_imu_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(Imu, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first IMU sample on %s — up to %.1fs", topic, timeout_s)
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


def _wait_for_odom(topic: str, timeout_s: float) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from nav_msgs.msg import Odometry
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("wheeltec_r550_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(Odometry, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first odom on %s — up to %.1fs", topic, timeout_s)
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
        except Exception:
            pass
    return seen.is_set()


# ── lifecycle: IMU primitive ──────────────────────────────────────────────
# NOTE: init_imu is deliberately a no-op.  IMU sentinel + declare are
# handled inside the chassis init() to avoid a race: init_imu could fire
# before init() has spawned imu_filter / base_serial, and then the wait
# for /imu_data would never succeed (timeout → Err).
#
# Keeping the IMU primitive as a separate capability so the contract
# surface stays clean (robonix/primitive/imu/imu), but all lifecycle
# work happens in one ordered sequence.

@imu_provider.on_init
def init_imu(cfg: dict):
    """No-op: IMU lifecycle is driven by chassis init()."""
    return Ok()


# ── lifecycle: chassis primitive ──────────────────────────────────────────

@provider.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE:

    1. Spawn all subprocesses in dependency order
    2. Wait for /odom_combined (EKF) + /imu_data (filtered IMU)
    3. Declare chassis and IMU primitive interfaces
    """
    odom_topic = "/" + cfg.get("odom_topic_name", "/odom_combined").lstrip("/")
    cmd_vel_topic = "/" + cfg.get("cmd_vel_topic", "/cmd_vel").lstrip("/")
    imu_topic = "/" + cfg.get("imu_topic", "/imu_data").lstrip("/")
    timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    # Spawn in dependency order:
    #   TFs (no deps)
    #   → joint_state_publisher (needs /robot_description from robot_desc primitive)
    #   → imu_filter (needs /imu/data_raw from base_serial)
    #   → base_serial (publishes /odom + /imu/data_raw)
    #   → ekf (needs /odom + /imu_data; publishes /odom_combined)
    try:
        _spawn_base_to_link(cfg)
        _spawn_base_to_gyro(cfg)
        _spawn_joint_state_publisher(cfg)
        _spawn_imu_filter(cfg)
        _spawn_wheeltec(cfg)
        _spawn_ekf(cfg)
    except Exception as e:
        _kill_all()
        return Err(f"spawn failed: {e}")

    # Wait for IMU first (imu_filter output arrives faster than EKF converges).
    if not _wait_for_imu(imu_topic, timeout):
        _kill_all()
        return Err(f"no IMU on {imu_topic} within {timeout:.1f}s")

    if not _wait_for_odom(odom_topic, timeout):
        _kill_all()
        return Err(f"no odometry on {odom_topic} within {timeout:.1f}s")

    # Declare both primitive interfaces from the chassis init (single
    # ordered path — no race between IMU and chassis primitives).
    imu_provider.declare_ros2_topic(
        "robonix/primitive/imu/imu",
        topic=imu_topic,
        qos="reliable",
    )
    provider.declare_ros2_topic(
        "robonix/primitive/chassis/odom",
        topic=odom_topic,
        qos="reliable",
    )
    provider.declare_ros2_topic(
        "robonix/primitive/chassis/twist_in",
        topic=cmd_vel_topic,
        qos="reliable",
    )
    log.info(
        "R550 chassis ready: odom=%s cmd_vel=%s imu=%s",
        odom_topic,
        cmd_vel_topic,
        imu_topic,
    )
    return Ok()


# ── lifecycle: shutdown ───────────────────────────────────────────────────

@imu_provider.on_shutdown
def shutdown_imu():
    log.info("stopping wheeltec R550 IMU (chassis driver will also be stopped)")
    _kill_all()
    return Ok()


@provider.on_shutdown
def shutdown():
    log.info("stopping wheeltec R550 chassis driver")
    _kill_all()
    return Ok()


# ── gRPC: chassis.move ────────────────────────────────────────────────────

@provider.grpc("robonix.primitive.chassis.move")
def move(req: "chassis_pb2.ExecuteMoveCommand_Request") -> "chassis_pb2.ExecuteMoveCommand_Response":
    """
    Primitive chassis motion command.

    NOT exposed as MCP.
    LLM should use:
        service/navigation/navigate

    Modes(priority):
        1. forward_m != 0:
            Move forward/backward specified distance (meter)

        2. rotate_deg != 0:
            Rotate in place specified angle (degree)

        3. velocity mode:
            Direct Twist velocity command for duration_sec
    """
    if cmd_vel_pub is None:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(
                data=json.dumps({
                    "error": "ROS2 not initialized"
                })
            )
        )
    from geometry_msgs.msg import Twist
    msg = req.command

    # =========================
    # Safety parameters
    # =========================

    DEFAULT_LINEAR_SPEED = 0.3  # m/s
    DEFAULT_ANGULAR_SPEED = 0.6  # rad/s
    DEFAULT_DURATION = 1.0  # sec

    MAX_LINEAR_SPEED = 0.5  # m/s
    MAX_ANGULAR_SPEED = 1.0  # rad/s

    CONTROL_PERIOD = 0.1  # sec

    speed_mps = float(
        os.environ.get(
            "TIAGO_CHASSIS_SPEED_MPS",
            DEFAULT_LINEAR_SPEED
        )
    )
    ang_speed_rps = float(
        os.environ.get(
            "TIAGO_CHASSIS_ANG_SPEED_RPS",
            DEFAULT_ANGULAR_SPEED
        )
    )
    default_duration = float(
        os.environ.get(
            "TIAGO_CHASSIS_CMD_DURATION_SEC",
            DEFAULT_DURATION
        )
    )
    # clamp configured speed
    speed_mps = min(
        abs(speed_mps),
        MAX_LINEAR_SPEED
    )
    ang_speed_rps = min(
        abs(ang_speed_rps),
        MAX_ANGULAR_SPEED
    )
    forward_m = float(
        getattr(msg, "forward_m", 0.0)
    )
    rotate_deg = float(
        getattr(msg, "rotate_deg", 0.0)
    )
    duration_sec = float(
        getattr(msg, "duration_sec", 0.0)
    )
    tw = Twist()
    mode = ""
    duration = 0.0
    # =========================
    # Mode 1:
    # distance movement
    # =========================
    if abs(forward_m) > 1e-6:
        sign = 1.0 if forward_m > 0 else -1.0
        tw.linear.x = sign * speed_mps
        duration = abs(forward_m) / speed_mps
        mode = "forward_m"
    # =========================
    # Mode 2:
    # rotation
    # =========================

    elif abs(rotate_deg) > 1e-6:

        rad = math.radians(rotate_deg)

        sign = 1.0 if rad > 0 else -1.0

        tw.angular.z = sign * ang_speed_rps

        duration = abs(rad) / ang_speed_rps

        mode = "rotate_deg"
    # =========================
    # Mode 3:
    # velocity control
    # =========================
    else:
        linear_x = float(msg.linear_x)
        linear_y = float(msg.linear_y)

        angular_z = float(msg.angular_z)
        # safety clamp
        tw.linear.x = max(
            min(linear_x, MAX_LINEAR_SPEED),
            -MAX_LINEAR_SPEED
        )
        tw.linear.y = linear_y
        tw.angular.z = max(
            min(angular_z, MAX_ANGULAR_SPEED),
            -MAX_ANGULAR_SPEED
        )
        duration = (
            duration_sec
            if duration_sec > 0
            else default_duration
        )
        mode = "velocity"
    stop = Twist()
    # =========================
    # Execute command
    # =========================

    try:
        start_time = time.time()
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            cmd_vel_pub.publish(tw)
            time.sleep(CONTROL_PERIOD)
    except Exception as e:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(
                data=json.dumps({
                    "error": str(e),
                    "mode": mode
                })
            )
        )
    finally:
        # emergency stop
        cmd_vel_pub.publish(stop)

    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(
            data=json.dumps({
                "status": "done",
                "mode": mode,
                "forward_m": forward_m,
                "rotate_deg": rotate_deg,
                "duration_sec": duration,
                "linear_x": tw.linear.x,
                "angular_z": tw.angular.z
            })
        )
    )


if __name__ == "__main__":
    provider.run()
