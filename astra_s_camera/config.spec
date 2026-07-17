# Runtime config accepted by the Orbbec Astra S camera primitive.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. It is documentation for deployers and tooling; the
# provider continues to parse and validate the values in its own code.

config:
  # string, default: /camera/color/image_raw.
  # Color (RGB) image topic published by the Astra S driver.
  # The sentinel waits for the first message on this topic before declaring
  # the primitive as ready.
  color_topic: /camera/color/image_raw

  # string, default: /camera/depth/image_raw.
  # Depth image topic (16UC1 in mm) published by the Astra S driver.
  depth_topic: /camera/depth/image_raw

  # string, default: /camera/color/camera_info.
  # Camera intrinsics topic (sensor_msgs/CameraInfo). Binds directly to
  # robonix/primitive/camera/intrinsics — the Astra S driver publishes
  # CameraInfo natively on this topic, no relay needed.
  intrinsics_topic: /camera/color/camera_info

  # string, default: camera_link.
  # Camera optical TF frame. Must match the camera link name in the URDF.
  cam_frame: camera_link

  # string, default: astra_camera.
  # ROS2 package containing the camera driver launch file.
  # The wheeltec_camera.launch.py wrapper simply includes astra.launch.xml
  # from this package; we launch it directly to remove the indirection.
  launch_package: astra_camera

  # string, default: astra.launch.xml.
  # Launch file inside launch_package. This is the Orbbec Astra S driver
  # entry point that publishes color + depth + IR image streams and the
  # depth point cloud.
  launch_file: astra.launch.xml

  # float, seconds, default: 60.0.
  # Maximum wait for the first color Image on color_topic before failing init.
  # USB cameras under a busy system may need >30 s to start streaming.
  sentinel_timeout_s: 60.0

