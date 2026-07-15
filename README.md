# Wheeltec R550 mini_tank — Robonix Primitives

三个原语分别管理机器人模型、底盘驱动和激光雷达，按依赖顺序启动。

## 原语概览

| 原语 | 命名空间 | 启动内容 |
|---|---|---|
| `robot_description` | `robonix/primitive/robot_description` | URDF 模型发布 |
| `chassis` | `robonix/primitive/chassis` | 底盘驱动 + IMU + EKF |
| `lidar` | `robonix/primitive/lidar` | LSLIDAR N10P 激光雷达 |

## 依赖关系

```
robot_description          ← 无依赖，必须最先启动
    │
    │  /robot_description (URDF)
    ▼
chassis                    ← 需要 URDF 才能运行 joint_state_publisher
    │                        发布的 TF: base_footprint → base_link
    │                                    base_footprint → gyro_link
    │  /odom_combined
    │  /imu_data
    ▼
lidar                      ← 需要 base_link 帧（chassis 提供）
```

## 各原语详细说明

### 1. robot_description — URDF 模型

**启动：** `ros2 launch turn_on_wheeltec_robot robot_mode_description_minibot.launch.py mini_tank:=true`

- 发布 `/robot_description`（URDF 参数，内部 RPC，不对外暴露 ROS2 topic）
- 启动 `robot_state_publisher`，配合 `joint_state_publisher` 构建完整 TF 树
- 通信方式：RPC，无 `declare_ros2_topic`

### 2. chassis — 底盘 + IMU

**启动：**

| 组件 | 命令 | 功能 |
|---|---|---|
| base_serial | `base_serial.launch.py` | 底盘 MCU 串口驱动 |
| base_to_link | `static_transform_publisher` | `base_footprint` → `base_link` (z=0.03715, mini_tank) |
| base_to_gyro | `static_transform_publisher` | `base_footprint` → `gyro_link` |
| joint_state_publisher | `joint_state_publisher` | 发布 fixed 关节状态（mini_tank 无活动关节） |
| imu_filter_madgwick | `imu_filter_madgwick_node` | `/imu/data_raw` → `/imu_data` (Madgwick 滤波) |
| wheeltec_ekf | `wheeltec_ekf.launch.py` | 融合 `/odom` + `/imu_data` → `/odom_combined` |

**对外暴露：**

| capability | 话题 | 方向 | QoS |
|---|---|---|---|
| `chassis/odom` | `/odom_combined` | out | reliable |
| `chassis/twist_in` | `/cmd_vel` | in | reliable |
| `imu/imu` | `/imu_data` | out | reliable |
| `chassis/move` | — | gRPC | — |

**话题数据流（内部+外部）：**

```
/dev/wheeltec_controller
        │
  base_serial.launch.py
        │
        ├── /odom ──────────────────────────┐
        │                                    │
        └── /imu/data_raw                    │
                │                            │
          imu_filter_madgwick                │
                │                            │
          /imu_data ─────────────────────────┤
                │                            │
                │                      wheeltec_ekf
                │                            │
                ▼                            ▼
    (对外) imu/imu               /odom_combined
                                        │
                                  (对外) chassis/odom
```

### 3. lidar — LSLIDAR N10P

**启动：** `ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py`

- 发布 `/scan`（`sensor_msgs/msg/LaserScan`），QoS RELIABLE，frame_id = `laser`
- 可选 extrinsics → 自动发布 `parent_frame → laser` 的 static TF
- 由于 `wheeltec_lidar.launch.py` 内无额外 TF publisher，若需 `base_link → laser` 变换，在 config 中配置 `extrinsics` 即可

**对外暴露：**

| capability | 话题 | 方向 | QoS |
|---|---|---|---|
| `lidar/lidar2d` | `/scan` | out | reliable |

## 启动顺序

在 `robonix_manifest.yaml` 中按依赖顺序排列：

```yaml
primitives:
  - robot_description   # 1st — 提供 URDF
  - chassis             # 2nd — 提供 TF + odom + imu
  - lidar               # 3rd — 提供 laser scan
```

## TF 树（完整运行时）

```
base_footprint
    ├── base_link                    (chassis: base_to_link)
    │       └── laser                (lidar: extrinsics, 可选)
    └── gyro_link                    (chassis: base_to_gyro)
```

- `base_footprint → base_link` 和 `base_footprint → gyro_link` 由 chassis 原语发布
- `base_link → laser` 由 lidar 原语的 `extrinsics` 配置发布（可选，用于 mapping/nav2 等消费者）
- URDF 中的 fixed 关节 TF 由 `robot_state_publisher` 根据 `/joint_states` 自动发布

