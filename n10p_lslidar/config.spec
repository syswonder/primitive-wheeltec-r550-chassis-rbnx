# Runtime config accepted by the LSLIDAR N10P lidar primitive.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. It is documentation for deployers and tooling; the
# provider continues to parse and validate the values in its own code.

config:
  # string, default: /scan.
  # LaserScan topic published by the lidar driver. The sentinel waits for the
  # first message on this topic before declaring the primitive as ready.
  scan_topic: /scan

  # string, default: laser.
  # TF frame_id written in the LaserScan header by the lidar driver.
  # Must match the frame name in the physical URDF so robot_state_publisher
  # can connect it into the TF tree.
  frame_id: laser

  # string, default: base_link.
  # Parent frame for the optional static_transform_publisher.
  # Only used when extrinsics is provided. Typically base_link (the chassis
  # centre frame published by the chassis primitive).
  parent_frame: base_link

  # dict, metres + radians, default: null.
  # Optional 6-DoF mount pose {x, y, z, roll, pitch, yaw} of the lidar in
  # parent_frame. When present, a static_transform_publisher is spawned
  # (parent_frame → frame_id). Omit when the chassis primitive or soma URDF
  # already publishes this edge — double-publishing causes TF_REPEATED_DATA.
  extrinsics: null

  # string, default: turn_on_wheeltec_robot.
  # ROS2 package containing the lidar launch file.
  launch_package: turn_on_wheeltec_robot

  # string, default: wheeltec_lidar.launch.py.
  # Launch file inside launch_package. Publishes /scan with QoS RELIABLE
  # and frame_id=laser; no additional TF publisher is included.
  launch_file: wheeltec_lidar.launch.py

  # float, seconds, default: 30.0.
  # Maximum wait for the first LaserScan on scan_topic before failing init.
  # LSLIDAR N10P typically starts streaming within 5–10 seconds.
  sentinel_timeout_s: 30.0

