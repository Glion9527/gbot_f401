# Link v2 串口契约（阶段 0）

本文是 STM32 ↔ 树莓派二进制链路的**唯一契约说明**。实现代码：

- 共享头文件：[protocol/link_v2.h](../protocol/link_v2.h)
- 几何/限速常量：[protocol/robot_geometry.h](../protocol/robot_geometry.h)
- 黄金帧：[protocol/tests/golden_frames.txt](../protocol/tests/golden_frames.txt)
- 宿主实现：[host/include/f401/drivers/link_v2.hpp](../host/include/f401/drivers/link_v2.hpp)、
  [host/src/drivers/link_v2.cpp](../host/src/drivers/link_v2.cpp)
- 宿主串口：[host/include/f401/drivers/serial_port.hpp](../host/include/f401/drivers/serial_port.hpp)

## 1. 为什么新增 v2，而不删掉 v1

现有 ASCII v1 协议（`@payload*CCCC\n`）已经稳定、可终端调试、有完整测试。
v2 的目标是学习与长期架构：固定长度、二进制、显式小端、带 seq/ARM、带宽更省。
因此策略是：

- **v1 冻结保留**：Mac 网关、Python 工具、`probe.py` 继续可用；
- **v2 新增**：先在树莓派 C++ 宿主上实现，固件侧下一步再加；
- 迁移完成前，两者不共用一个串口会话；固件可用编译开关选择，后续再做
  “首个合法帧决定模式”的运行时切换。

## 2. 现有 ASCII v1 解析代码在哪里

| 位置 | 内容 |
| --- | --- |
| [Core/Src/remote_control.c:89](../Core/Src/remote_control.c#L89) | `CRC16()`：CRC16-CCITT-FALSE，与 Python 一致 |
| [Core/Src/remote_control.c:293](../Core/Src/remote_control.c#L293) | `ParseFrame()`：校验 `@正文*XXXX` 结构、长度、CRC |
| [Core/Src/remote_control.c:199](../Core/Src/remote_control.c#L199) | `Execute()`：H/A/V/S 命令解释与 ACK |
| [Core/Src/remote_control.c:460](../Core/Src/remote_control.c#L460) | `RemoteControl_Poll()`：主循环组帧状态机、遥测发送、喂狗 |
| [Core/Src/remote_control.c:397](../Core/Src/remote_control.c#L397) | `RemoteControl_UART_RxCplt()`：USART2 中断逐字节入环形缓冲 |
| [Core/Src/remote_control.c:440](../Core/Src/remote_control.c#L440) | `RemoteControl_Tick10ms()`：300ms 超时/按键/故障撤销使能 |
| [host_control/robot_link/protocol.py:12](../host_control/robot_link/protocol.py#L12) | Python `crc16()`，标准向量 `123456789 -> 0x29B1` |
| [host_control/robot_link/protocol.py:24](../host_control/robot_link/protocol.py#L24) | `serial_frame()`：`@payload*CCCC\n` 编码 |
| [host_control/robot_link/protocol.py:33](../host_control/robot_link/protocol.py#L33) | `serial_payload()`：边界/长度/CRC 校验 |
| [host_control/robot_link/protocol.py:65](../host_control/robot_link/protocol.py#L65) | `parse_serial()`：ACK/T 回包解析 |
| [tests/remote_control/test_remote_control.c.txt](../tests/remote_control/test_remote_control.c.txt) | 固件侧 C 主机仿真测试 |
| [host_control/tests/test_robot_link.py](../host_control/tests/test_robot_link.py) | Python 编解码/网关测试 |
| [docs/remote-control-protocol.md](remote-control-protocol.md) | v1 完整协议文档 |

ROS 2 侧不直接解析 ASCII：`ros2_ws` 通过 `robot_link` 客户端走 TCP JSON，
再由 Mac 网关转成串口帧。

## 3. 帧格式

统一约定：

- 多字节整数**小端**；
- 速度单位 **mm/s**（int16，带符号）；转速单位 **RPM×10**；
- 轮序固定 **FL、FR、RL、RR**，逻辑正方向都是“推动小车前进”；
- 不要对整个结构体 `memcpy`，逐字段 pack/unpack（padding/对齐会坑）。

### 3.1 CMD（树莓派 → STM32，18 字节，建议 50Hz）

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | header0 | u8 | `0xA5` |
| 1 | header1 | u8 | `0x5A` |
| 2 | version | u8 | `0x02` |
| 3 | type | u8 | `0x01` |
| 4 | seq | u16 LE | 命令序号，回绕比较 |
| 6 | flags | u16 LE | bit0 ARM、bit1 DISARM、bit2 ESTOP |
| 8 | fl_mm_s | i16 LE | 左前轮目标 |
| 10 | fr_mm_s | i16 LE | 右前轮目标 |
| 12 | rl_mm_s | i16 LE | 左后轮目标 |
| 14 | rr_mm_s | i16 LE | 右后轮目标 |
| 16 | crc16 | u16 LE | 覆盖 offset 2..15 |

### 3.2 TELEM（STM32 → 树莓派，30 字节，建议 50Hz）

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | header0 | u8 | `0xA5` |
| 1 | header1 | u8 | `0x5A` |
| 2 | version | u8 | `0x02` |
| 3 | type | u8 | `0x81` |
| 4 | seq | u16 LE | 最近接受的 CMD seq（回显） |
| 6 | mcu_ms | u16 LE | MCU 毫秒时钟（回绕） |
| 8 | status | u16 LE | ARMED/FAULT/ESTOP/CMD_TIMEOUT |
| 10 | fault_mask | u16 LE | 四轮故障按位或 |
| 12 | delta | 4×i16 LE | 自上一 TELEM 以来的编码器增量 |
| 20 | rpm_x10 | 4×i16 LE | 四轮转速 ×10 |
| 28 | crc16 | u16 LE | 覆盖 offset 2..27 |

## 4. CRC 与重同步

- CRC16-CCITT-FALSE：初值 `0xFFFF`、多项式 `0x1021`、不反射、无 xorout；
- 标准向量：`"123456789"` → `0x29B1`（固件、Python、C++ 三端一致）；
- 接收端按固定长度取帧；长度/版本/类型/CRC 任一不符，**只丢 1 字节**再找
  `0xA5 0x5A`，这样半帧、拔插线残留不会卡死状态机；
- 黄金帧见 [protocol/tests/golden_frames.txt](../protocol/tests/golden_frames.txt)。

## 5. 语义与安全

- **seq**：u16 回绕；接收端只接受比上次更新的序号，重放/旧序号丢弃；
- **ARM**：必须显式置位；STM32 仍需满足“无故障、按钮松开、四轮停稳 200ms、
  250ms 静默”等现有条件才允许运动；DISARM/超时立即停并撤销使能；
- **超时**：`LINK_V2_CMD_TIMEOUT_MS = 300`；沿用现有固件的 300ms 遥控超时和
  500ms 电机看门狗，不另造一套数字；
- **遥测**：50Hz（20ms）；TELEM 超过 300ms 没更新，宿主里程计返回 `nullopt`；
- **速度限幅**：宿主先按 vx≤300、vy≤150、wz≤600mrad/s 限幅，再对四轮做
  75RPM 等比例限幅；STM32 侧仍要保留 110RPM 硬上限（纵深防御）。

## 6. 黄金帧

| 名称 | 字节 |
| --- | --- |
| `cmd_arm_zero` | `A5 5A 02 01 01 00 01 00 00 00 00 00 00 00 00 00 FB 76` |
| `cmd_drive_300` | `A5 5A 02 01 02 00 01 00 2C 01 2C 01 2C 01 2C 01 86 29` |
| `cmd_mixed` | `A5 5A 02 01 03 00 01 00 38 FF 64 00 00 00 32 00 28 B7` |
| `telem_nominal` | `A5 5A 02 81 02 00 D2 04 01 00 00 00 0A 00 EC FF 1E 00 D8 FF 2C 01 2C 01 2C 01 2C 01 36 DC` |

## 7. 当前实现状态与下一步

- ✅ 宿主：`SerialPort` RAII、v2 编解码、`SerialBase`、`SimBase`、PTY 测试；
- ⏳ 固件：尚未实现 v2 接收。下一步新增
  `Core/Inc/link_v2.h` / `Core/Src/link_v2.c`，复用本文契约；
  解析在主循环，超时检查挂到 TIM9 的 10ms 回调；
  v1 ASCII 代码路径保持不动，用编译开关选择；
- 联调顺序：先 `--dry-run` 核对黄金帧 → 固件烧 v2 → `--port` 收 TELEM →
  手推小车看里程计 → 再进入阶段 1 的运动学库。

## 8. 构建与测试

```bash
cd host
make
make test        # 编解码 + 运动学 + PTY 串口 + C 头文件检查
make sanitize    # ASan/UBSan（独立 build/sanitize 目录）
./build/f401_host --dry-run
./build/f401_host --sim
./build/f401_host --port /dev/ttyACM0 --arm
```

> PTY 测试在受限沙箱里会打印 SKIP；在 Linux/树莓派或普通终端会完整执行。
