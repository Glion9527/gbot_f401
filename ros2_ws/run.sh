#!/usr/bin/env bash
# 在 Ubuntu 中执行；自动找到同一仓库的纯 Python 网络客户端。
# 例：bash ros2_ws/run.sh network_bridge --ros-args -p host:=Mac的IP地址
# set -e遇到命令失败退出，-u检查未定义变量，pipefail让管道前段失败也能被发现。
set -euo pipefail
# 根据脚本自身的位置定位工作区，所以从仓库根目录或其他目录启动均可找到install。
task_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo '未找到 /opt/ros/jazzy/setup.bash，请先安装 ROS 2 Jazzy。' >&2
  exit 1
fi
if [[ ! -f "$task_ws/install/setup.bash" ]]; then
  echo '请先在 ros2_ws 中执行 colcon build --symlink-install。' >&2
  exit 1
fi
# ROS环境脚本可能引用未定义变量，因此source期间暂时关闭-u，完成后恢复。
# 先加载系统ROS，再叠加本工程colcon生成的环境，使ros2能发现f401_robot_bridge包。
set +u
source /opt/ros/jazzy/setup.bash
source "$task_ws/install/setup.bash"
set -u
# robot_link位于工作区旁边的host_control目录，并不在ROS Python包内部。
# 把该目录加入模块搜索路径后，桥接节点才能import robot_link.client。
export PYTHONPATH="$task_ws/../host_control${PYTHONPATH:+:$PYTHONPATH}"
# "$@"原样保留所有参数；第一个参数是network_bridge/keyboard/keyboard_window。
# exec用ROS进程替换当前shell，使退出码和Ctrl-C信号直接传到实际节点进程。
exec ros2 run f401_robot_bridge "$@"
