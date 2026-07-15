# Runtime config accepted by the robot_description primitive.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. It is documentation for deployers and tooling; the
# provider continues to parse and validate the values in its own code.

config:
  # string, default: mini_tank.
  # Robot model name passed to robot_mode_description_minibot.launch.py.
  # Supported minibot values: mini_tank, mini_mec, mini_akm, mini_4wd,
  # mini_diff, mini_omni. Flagship chassis must use
  # robot_mode_description.launch.py instead (requires code change).
  robot_model: mini_tank

  # float, seconds, default: 30.0.
  # Maximum wait for /robot_description to appear after spawning the launch.
  # Increase on resource-constrained boards where URDF loading is slow.
  sentinel_timeout_s: 30.0

