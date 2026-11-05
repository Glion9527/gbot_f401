#!/usr/bin/env bash
# 在 Ubuntu 中执行；自动找到同一仓库的纯 Python 网络客户端。
set -euo pipefail
task_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo '未找到 /opt/ros/jazzy/setup.bash，请先安装 ROS 2 Jazzy。' >&2
  exit 1
fi
if [[ ! -f "$task_ws/install/setup.bash" ]]; then
  echo '请先在 ros2_ws 中执行 colcon build --symlink-install。' >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$task_ws/install/setup.bash"
set -u
export PYTHONPATH="$task_ws/../host_control${PYTHONPATH:+:$PYTHONPATH}"
exec ros2 run f401_robot_bridge "$@"
