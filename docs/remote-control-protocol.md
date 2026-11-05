# 底盘遥控接口 v1

此接口由 STM32 固件、Mac 串口中转和 Ubuntu/ROS 2 客户端共同实现。内部轮序固定为 FL、FR、RL、RR；物理接口为 A、D、B、C。上电、连接、重连均不自动运动。默认遥控模式为 ROBOT_APP_MODE=7，旧的 0..6 模式保留。

## 串口

115200、8N1、无流控。ASCII 行格式为 `@payload*CCCC\n`，允许行尾 CRLF；CCCC 为 payload 字节的 CRC16-CCITT-FALSE（初值 0xFFFF，多项式 0x1021，大写 4 位十六进制）。数字均十进制整数；session 和 seq 是 uint32。命令行最多 127 字节，不含行尾。

| payload | 作用 |
| --- | --- |
| `H,session,0` | 新会话，立即停止并解除使能，重置序号；ACK 序号 0 |
| `A,session,seq` | 显式使能：无故障、按钮松开、四轮停稳至少 250ms 才接受 |
| `V,session,seq,vx,vy,wz` | vx/vy 单位 mm/s，wz 单位 mrad/s；均为车身坐标，前、左、俯视逆时针为正 |
| `S,session,seq` | 停止且解除使能；下一次运动需重新 A |

H 使用非零 session；其余命令必须匹配当前 session，seq 必须比上次已消费序号新（uint32 回绕比较）。结构正确且序号新的命令消费序号，即使因状态被拒绝也不允许重播。V 的轴向输入限制为 abs(vx),abs(vy)<=150，abs(wz)<=600。固件再对四轮目标等比例限幅到 60RPM；几何参数沿用 D=80mm、L=185mm、W=220mm。非零 V 仅在已使能时接受；零 V 停止但保持使能。换向若不满足 motor4 的停止/静止条件，停止、解除使能并返回 REJECTED，不自动排队换向。

ACK payload 为 `ACK,session,seq,result`，result 取 `OK`、`DISARMED`、`BUSY`、`FAULT`、`REJECTED`、`INVALID`。没有可验证 session/seq 的坏帧不需回复。CRC 错误、接收溢出/串口错误/过长帧在使能状态下停止并解除使能，不刷新运动许可。不得将半帧跨超时重新拼成运动指令。

使能后 300ms 内必须收到新的有效 V（零速度亦可）。H/A/S、状态查询、重复序号、坏帧均不延续 V 许可；A 仅提供首次 V 的 300ms 起步窗口。超时、按下蓝色 USER、任一电机故障立即停止并解除使能。超时检查由 10ms 控制中断执行，不依赖 UART 主循环。已有 motor4 的 500ms 看门狗继续保留，只有当前有效且未过期的遥控许可才允许应用喂狗。USART 接收中断只入队，主循环解析/发送日志。

固件每 100ms 发送一条带同样 CRC 的遥测：

`T,session,mcu_ms,armed,last_seq,fault_mask,cnt_fl,cnt_fr,cnt_rl,cnt_rr,rpm10_fl,rpm10_fr,rpm10_rl,rpm10_rr,target10_fl,target10_fr,target10_rl,target10_rr,pwm_fl,pwm_fr,pwm_rl,pwm_rr`

cnt 用 64 位十进制打印；rpm10/target10 为 RPM 的十倍；PWM 为有符号千分比。fault_mask 为四轮故障的按位或。session 未建立时为 0。上位机必须检查遥测时效和 session；不可把旧缓存当成当前控制许可。遥测长度上限为 511 字节。

## Mac ↔ Ubuntu

TCP 端口默认 8765。UTF-8 JSON，每行一条，最长 4096 字节；单一控制客户端。Mac 启动创建本地随机 token 文件（忽略进 Git，用户权限），Ubuntu 用相同 token 配对。不使用硬编码公共口令，不输出 token 到测试日志。

客户端首帧 `{"type":"auth","token":"..."}`；服务器建立新串口 H 会话且确认后返回 `{"type":"welcome","version":1,"challenge":"...","telemetry":...}`。每连接均重新握手，连接/重连不会 ARM。认证失败关闭连接。

后续请求含 type 和上次回复的 challenge，type 为 `arm`、`velocity`、`stop`、`status`。velocity 额外含整数 vx_mm_s、vy_mm_s、wz_mrad_s。服务器每次回复 `{"type":"reply","ok":true/false,"reason":"...","challenge":"...","telemetry":...}`。challenge 为一次性随机值，300ms 有效；过期、重复、错误值或超长/坏 JSON 时解除使能并结束控制会话。该机制拒绝长延迟/排队的旧网络指令，不比较两台机器不一致的时钟。

telemetry 使用对象：`session`, `mcu_ms`, `armed`(bool), `last_seq`, `fault_mask`, `wheels`（依次 FL/FR/RL/RR 的 4 个对象，各含 `id`,`count`,`rpm_x10`,`target_x10`,`pwm`），以及 Mac 添加的 `age_ms`。串口 ACK 未确认、遥测过期 500ms、session 不匹配不能报告 arm/velocity 成功。无新网络运动指令不得持续重发旧 V。断线/退出尽力发 S；即使来不及发送，STM32 也按本地 300ms 超时停止。发生错误后不自动重新使能。

客户端采用顺序请求/响应，最多 20Hz；socket 请求超时后断开，不重发该运动请求。Ubuntu ROS 桥接还必须单独检查 cmd_vel 的时效（本地单调时钟），过期发送零 V；不能仅因 TCP 仍通就反复延续上一条非零 ROS 指令。网络故障时须新的显式使能操作才恢复。初始阶段只发布四轮状态和连接状态，不将未标定数据伪装为完整导航里程计。
