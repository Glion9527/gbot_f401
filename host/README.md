# host：树莓派/Mac 的 C++20 宿主骨架

这是“契约 + C++ 宿主骨架”的第一版，对应阶段 0/3 的最小切片：

```text
protocol/                共享契约（C 兼容头文件 + 黄金帧）
host/
├── include/f401/
│   ├── core/            纯算法：mecanum 运动学、里程计（无硬件依赖）
│   ├── hal/             IBase 接口（setVelocity/getOdometry/poll）
│   ├── drivers/         SerialPort RAII、Link v2 编解码、SerialBase
│   └── app/             SimBase 仿真后端
├── src/                 实现与 main
├── tests/               编解码、串口（PTY 假串口）、运动学/里程计测试
├── Makefile             本机构建（macOS/Linux，无需 CMake）
└── CMakeLists.txt       树莓派/后续 CMake 构建
```

## 构建与测试

```bash
cd host
make            # 生成 build/f401_host
make test       # 编解码 + 运动学 + PTY 串口 + C 头文件检查
make sanitize   # ASan/UBSan 版本（可选）
```

有 CMake 时：

```bash
cmake -S host -B host/build
cmake --build host/build
ctest --test-dir host/build --output-on-failure
```

## 运行

```bash
./build/f401_host --dry-run          # 打印一帧 CMD 的十六进制
./build/f401_host --sim              # SimBase 仿真 2 秒并打印位姿
./build/f401_host --port /dev/ttyACM0 --arm --vx 0.05
./build/f401_host --port /dev/ttyACM0 --imu   # 每秒打印 TELEM 中的 IMU 原始值
```

> 固件侧已实现 Link v2（`Core/Src/link_v2.c`，`LINK_PROTOCOL_V2=1`）。
> 烧录后 `--port` 能收到 50Hz TELEM；`--vx/--vy/--wz` 以 m/s、rad/s 指定目标。
> `--imu` 打印 `imu_flags`、加速度 mg、陀螺 0.1dps、磁场 0.1uT、温度 0.1C。
> 不烧 v2、仍运行 ASCII v1 时，宿主只能发帧、收不到 TELEM。

## 设计边界

- `core/` 不 include 串口、ROS、线程；可以在 Mac 上独立测试。
- `hal/` 只有接口；`SerialBase` / `SimBase` 是可替换实现。
- `drivers/` 负责字节流与协议；不关心“去哪个房间”。
- 单线程骨架：主循环调用 `poll()`；线程化留给阶段 2。

## 黄金帧

`protocol/tests/golden_frames.txt` 是 C/C++/Python 三端共用的字节级契约。
改动帧格式时，先改 `protocol/link_v2.h` 和黄金帧，再同步固件与宿主实现。
