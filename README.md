# Wheeltec R550 mini\_tank — Robonix Deploy Primitives

A full Robonix deployment for the **Wheeltec R550 mini\_tank**: 2D SLAM mapping
with N10P LiDAR + Orbbec Astra S RGB-D camera, rtabmap, Nav2 navigation, and VLM
pilot. All components boot via `rbnx boot` from a single `robonix_manifest.yaml`.

## Package inventory

| Type | Name | Path | Provider ID | Summary |
|------|------|------|-------------|---------|
| **primitive** | robot\_desc | `./primitive-wheeltec-r550-chassis-rbnx/robot_description` | `robot_description` | URDF model + robot\_state\_publisher |
| **primitive** | chassis | `./primitive-wheeltec-r550-chassis-rbnx/r550_chassis` | `r550_chassis` | Chassis driver + IMU + EKF |
| **primitive** | lidar | `./primitive-wheeltec-r550-chassis-rbnx/n10p_lslidar` | `n10p_lslidar` | LSLIDAR N10P 2D laser scanner |
| **primitive** | camera | `./astra_s_camera` | `astra_s_camera` | Orbbec Astra S RGB-D depth camera |
| **service** | mapping | `./service-map-rbnx` | `mapping` | rtabmap SLAM — occupancy grid, point cloud, pose, odom |
| **service** | nav2 | `./service-navigation-rbnx` | `nav2` | Nav2 goal-based navigation |
| **system** | scene | `package_manifest.jetson-native.yaml` | `scene` | Semantic scene understanding (pins camera) |

### System layer

| Component | Port | Role |
|-----------|------|------|
| atlas | `50051` | Capability registry |
| executor | `50061` | Tool-call dispatcher |
| pilot | `50071` | VLM brain (OpenAI-format upstream) |
| liaison | `50081` | External API bridge |
| soma | `50091` | Robot body description + health |
| vitals | `50093` | Dashboard / system monitoring |
| scene | `50107` | Semantic scene graph (camera consumer) |

## Dependency graph

```
robot_description                     ← no deps, must boot first
    │  /robot_description (URDF)
    ▼
chassis                               ← needs URDF for joint_state_publisher
    │  publishes: base_footprint → base_link, base_footprint → gyro_link
    │  /odom_combined  /imu_data  /cmd_vel
    │
    ├──► lidar                        ← needs base_link TF
    │       /scan (LaserScan, frame_id=laser)
    │
    └──► camera                       ← needs camera_link TF from URDF
            /camera/color/image_raw   /camera/depth/image_raw
            /camera/color/camera_info
            + snapshot MCP tools
            │
            ├──► mapping              ← lidar2d (n10p_lslidar) + odom (r550_chassis)
            │       /map (OccupancyGrid)  /rtabmap/cloud_map  /rtabmap/odom
            │
            ├──► nav2                 ← map (mapping) + odom (r550_chassis) + scan (n10p_lslidar)
            │       service/navigation/{navigate, status, cancel}
            │
            └──► scene                ← camera (astra_s_camera)
                    system/scene/{list_objects, goal_near}
```

## Primitives

### 1. robot\_description — URDF model

**Launch:** `ros2 launch turn_on_wheeltec_robot robot_mode_description_minibot.launch.py mini_tank:=true`

| Capability | Transport | Topic / Notes |
|------------|-----------|---------------|
| `robonix/primitive/robot_description/driver` | gRPC | Lifecycle gate |

Publishes `/robot_description` (TRANSIENT\_LOCAL) and spawns
`robot_state_publisher` so the full TF tree is available to downstream consumers.

### 2. chassis — chassis driver + IMU + EKF

**Launch:** `base_serial.launch.py` + `wheeltec_ekf.launch.py`

| Capability | Transport | Topic | QoS |
|------------|-----------|-------|-----|
| `robonix/primitive/chassis/driver` | gRPC | — | — |
| `robonix/primitive/chassis/odom` | topic\_out | `/odom_combined` (EKF-fused) | reliable |
| `robonix/primitive/chassis/move` | gRPC | — | — |
| `robonix/primitive/chassis/twist_in` | topic\_in | `/cmd_vel` | reliable |
| `robonix/primitive/imu/driver` | gRPC | — | — |
| `robonix/primitive/imu/imu` | topic\_out | `/imu_data` (Madgwick-filtered) | reliable |

TF published: `base_footprint → base_link` (z=0.03715), `base_footprint → gyro_link`.

Data flow (internal + external):

```
/dev/wheeltec_controller
        │
  base_serial.launch.py
        │
        ├── /odom ──────────────────────┐
        │                               │
        └── /imu/data_raw               │
                │                       │
          imu_filter_madgwick           │
                │                       │
          /imu_data ────────────────────┤
                │                       │
                │                 wheeltec_ekf
                │                       │
                ▼                       ▼
      (external) imu/imu    /odom_combined
                                   │
                             (external) chassis/odom
```

### 3. lidar — LSLIDAR N10P

**Launch:** `ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py`

| Capability | Transport | Topic | QoS |
|------------|-----------|-------|-----|
| `robonix/primitive/lidar/driver` | gRPC | — | — |
| `robonix/primitive/lidar/lidar` | topic\_out | `/scan` | reliable |

Publishes `sensor_msgs/LaserScan` with `frame_id=laser`. The
`base_link → laser` TF edge comes from the URDF via `robot_state_publisher`.

**Self-heal:** 3 retry attempts if no LaserScan appears within the sentinel
timeout — recovers from N10P stream-start failures without a manual power cycle.

### 4. camera — Orbbec Astra S RGB-D

**Launch:** `ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py`

| Capability | Transport | Topic | QoS |
|------------|-----------|-------|-----|
| `robonix/primitive/camera/driver` | gRPC | — | — |
| `robonix/primitive/camera/rgb` | topic\_out | `/camera/color/image_raw` | reliable |
| `robonix/primitive/camera/depth` | topic\_out | `/camera/depth/image_raw` | reliable |
| `robonix/primitive/camera/intrinsics` | topic\_out | `/camera/color/camera_info` | reliable |
| `robonix/primitive/camera/snapshot` | MCP | — | one-shot RGB JPEG |
| `robonix/primitive/camera/depth_snapshot` | MCP | — | one-shot depth JPEG |
| `robonix/primitive/camera/extrinsics` | — | — | declared only (URDF-sourced) |

The camera subscribes to RGB and depth topics with permanent subscriptions
(no transient ROS nodes at call time). snapshot / depth\_snapshot return the
latest cached frame as a JPEG-encoded `sensor_msgs/Image`.

Intrinsics binds directly to the Astra S native CameraInfo topic — no relay
needed. Extrinsics are declared in `package_manifest.yaml` but not implemented:
the URDF + `robot_state_publisher` already provide the `base_link → camera_link`
TF edge.

**Astra S native topics (published by wheeltec\_camera.launch.py):**

| Topic | Type |
|-------|------|
| `/camera/color/image_raw` | sensor\_msgs/Image |
| `/camera/color/camera_info` | sensor\_msgs/CameraInfo |
| `/camera/depth/image_raw` | sensor\_msgs/Image |
| `/camera/depth/camera_info` | sensor\_msgs/CameraInfo |
| `/camera/depth/points` | sensor\_msgs/PointCloud2 |
| `/camera/ir/image_raw` | sensor\_msgs/Image |
| `/camera/ir/camera_info` | sensor\_msgs/CameraInfo |

## Services

### mapping — rtabmap SLAM

| Capability | Transport | Topic / Notes |
|------------|-----------|---------------|
| `robonix/service/map/driver` | gRPC | Lifecycle |
| `robonix/service/map/occupancy_grid` | topic\_out | `/map` (OccupancyGrid) |
| `robonix/service/map/pointcloud` | topic\_out | `/rtabmap/cloud_map` |
| `robonix/service/map/pose` | topic\_out | `/robonix/map/pose` |
| `robonix/service/map/odom` | topic\_out | `/rtabmap/odom` |

Sensor providers: `lidar2d → n10p_lslidar`, `odom → r550_chassis`.
Web UI at `http://<robot-ip>:8091`.

### nav2 — goal-based navigation

| Capability | Transport | Topic / Notes |
|------------|-----------|---------------|
| `robonix/service/navigation/driver` | gRPC | Lifecycle |
| `robonix/service/navigation/navigate` | gRPC + MCP | Start a NavigateToPose run |
| `robonix/service/navigation/navigate/status` | gRPC + MCP | Poll run state |
| `robonix/service/navigation/navigate/cancel` | gRPC + MCP | Cancel active run |

Provider pins: `map → mapping`, `odom → r550_chassis`, `scan → n10p_lslidar`.
At `Driver(CMD_INIT)`: resolves Atlas providers, materializes the deploy-owned
Nav2 YAML, spawns nav2\_bringup, waits for `navigate_to_pose` action server,
then declares capabilities. Missing required providers return `deferred`.

The deploy-owned `config/nav2_params.yaml` must be created next to
`robonix_manifest.yaml` — copy and tune
`service-navigation-rbnx/config/nav2_params.example.yml`.

## TF tree (full runtime)

```
base_footprint
    ├── base_link                          (chassis: base_to_link, z=0.03715)
    │       ├── laser                      (URDF via robot_state_publisher)
    │       └── camera_link                (URDF via robot_state_publisher)
    └── gyro_link                          (chassis: base_to_gyro)
```

## Boot order

```yaml
system:    atlas → soma → executor → pilot → liaison → vitals → scene
primitive: robot_desc → chassis → lidar → camera
service:   mapping → nav2
```

Primitives boot in dependency order. Services use the Atlas defer-queue:
nav2 waits for mapping, which waits for lidar + chassis.

## Environment

Propagated by `rbnx boot` from the `env:` block in `robonix_manifest.yaml`:

| Variable | Used by | Notes |
|----------|---------|-------|
| `VLM_BASE_URL` | pilot | LLM upstream endpoint |
| `VLM_API_KEY` | pilot | LLM API key |
| `VLM_MODEL` | pilot | Model name |
| `LOG` | all | Log level (default `INFO`) |
| `ROS_DISTRO` | primitives | ROS 2 distro (default `humble`) |

Pre-export these in your shell or replace `${VLM_*}` with literal values in the
manifest.

## Host prerequisites

- ROS 2 Humble sourced at `/opt/ros/humble/setup.bash`
- `turn_on_wheeltec_robot` workspace on the ROS package path
- Nav2 packages (`ros-humble-nav2-bringup` + plugins)
- `rbnx` CLI on PATH
- Python 3.10+ with `numpy`, `PIL` (Pillow), `grpcio`

## Quickstart

```bash
# 1. Build all packages
rbnx build

# 2. Boot the full stack
rbnx boot

# 3. Open the mapping web UI
# http://<robot-ip>:8091
```

## License

Apache-2.0 (matches robonix upstream).
