# f401_base：STM32 Link v2 的 ROS 2 C++ 节点

直接占用 `/dev/f401`，不再经过 `mac_gateway.py`、TCP 或配对码。

## 接口

| 方向 | 名称 | 类型 |
|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| 发布 | `/odom` | `nav_msgs/msg/Odometry` |
| 发布 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 发布 | `/robot/state` | `std_msgs/msg/String`（JSON） |
| 服务 | `/robot/arm` | `std_srvs/srv/Trigger` |
| 服务 | `/robot/disarm` | `std_srvs/srv/Trigger` |
| 服务 | `/robot/connect` | `std_srvs/srv/Trigger` |
| TF | `odom -> base_link` | `tf2` |

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
ros2 topic echo /robot/state --once
ros2 run tf2_ros tf2_echo odom base_link
```

## 注意

- 节点启动时调用 `SerialBase::syncSequence()`，把本地 seq 对齐到 STM32 的 `last_seq`；
  否则新进程的 seq=1 会被当成旧序号，ARM 永远失败。
- 同一个 `/dev/f401` 只能有一个所有者：不要同时运行 `f401_host`、`mac_gateway.py`、
  `network_bridge`。
- 当前 `/cmd_vel` 只接受 `TwistStamped`，与现有 `keyboard` 节点一致；
  `teleop_twist_keyboard` 发布的是 `Twist`，暂不兼容。
