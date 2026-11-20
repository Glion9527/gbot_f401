# f401_base：STM32 Link v2 的 ROS 2 C++ 节点

直接占用 `/dev/f401`，不再经过 `mac_gateway.py`、TCP 或配对码。

## 接口

| 方向 | 名称 | 类型 |
|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| 发布 | `/odom` | `nav_msgs/msg/Odometry` |
| 发布 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 发布 | `/imu/data_raw` | `sensor_msgs/msg/Imu`（原始值，无姿态） |
| 发布 | `/imu/mag` | `sensor_msgs/msg/MagneticField` |
| 发布 | `/robot/state` | `std_msgs/msg/String`（JSON） |
| 服务 | `/robot/arm` | `std_srvs/srv/Trigger` |
| 服务 | `/robot/disarm` | `std_srvs/srv/Trigger` |
| 服务 | `/robot/connect` | `std_srvs/srv/Trigger` |
| TF | `odom -> base_link` | `tf2`（动态） |
| TF | `base_link -> imu_link` | `tf2`（静态，当前单位变换） |

## 构建

```bash
cd ~/code/gbot_f401/ros2_ws
colcon build --symlink-install --packages-select f401_base
source install/setup.bash
```

## 运行

先确保没有旧网关/桥接占用串口：

```bash
pkill -f mac_gateway.py
pkill -f network_bridge
```

启动节点：

```bash
ros2 launch f401_base f401_base.launch.py
```

另一个终端用现有键盘（发布 `TwistStamped`，调用 `/robot/arm`）：

```bash
cd ~/code/gbot_f401
bash ros2_ws/run.sh keyboard --ros-args -p linear_speed:=0.08
```

检查：

```bash
ros2 topic hz /odom
ros2 topic hz /imu/data_raw
ros2 topic echo /robot/state --once
ros2 topic echo /imu/data_raw --once
ros2 topic echo /imu/mag --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

## 键盘失焦

终端版 `keyboard` 失焦后 **保持当前档位和 ARM**，所以可以切到另一个终端执行
`ros2 topic hz /odom` 等验收命令。需要停止时：

- 回到键盘终端按空格；
- 或在任意终端执行：

```bash
ros2 service call /robot/disarm std_srvs/srv/Trigger
```

`keyboard_window` 是独立的 Tk 窗口版，焦点策略不同，不用于本次验收。

## 注意

- 节点启动时调用 `SerialBase::syncSequence()`，把本地 seq 对齐到 STM32 的 `last_seq`；
  否则新进程的 seq=1 会被当成旧序号，ARM 永远失败。
- 同一个 `/dev/f401` 只能有一个所有者：不要同时运行 `f401_host`、`mac_gateway.py`、
  `network_bridge`。
- 当前 `/cmd_vel` 只接受 `TwistStamped`，与现有 `keyboard` 节点一致；
  `teleop_twist_keyboard` 发布的是 `Twist`，暂不兼容。
- `/imu/data_raw` 的 `orientation_covariance[0] = -1`，表示没有姿态；
  `/odom` 仍然只来自轮速，IMU 尚未参与融合。
- `base_link -> imu_link` 的安装参数：

  | 参数 | 含义 |
  |---|---|
  | `imu_x/y/z` | IMU 中心相对 base_link 的平移，m |
  | `imu_roll/pitch/yaw` | IMU 相对 base_link 的旋转，rad |

  `base_link` 当前定义为四轮中心的地面投影点：x 向前、y 向左、z 向上。
  因此 `imu_z` 是 IMU 中心离地高度，`imu_x/y` 是 IMU 中心相对四轮中心的
  前后/左右偏移。

  节点每秒按当前参数重发静态 TF，所以可以先启动后用 `ros2 param set` 现场调，
  确认后把值写进 [launch/f401_base.launch.py](launch/f401_base.launch.py)。
