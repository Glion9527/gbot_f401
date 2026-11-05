"""与 ROS 分离的运动许可：只保存新鲜指令，连接/使能都不复用旧目标。"""
# 本模块只处理数值与状态，不导入rclpy，便于在普通Python环境中验证边界情况。
import math


def fault_description(mask):
    """硬件故障与网络请求成功是两个状态，避免状态查询OK掩盖故障。"""
    # fault_mask是位掩码，可以同时置多个位；使用按位与逐一翻译成中文原因。
    names = [(1, '200ms无编码器反馈'), (2, '反馈方向相反'),
             (4, '电机看门狗超时'), (8, '参数配置错误'), (16, '外设启动失败')]
    reasons = [name for bit, name in names if mask & bit]
    if mask & ~31:
        reasons.append('未知故障位')
    return 'fault_mask=%d（%s）' % (mask, '、'.join(reasons))


class CommandState:
    def __init__(self, timeout=0.25):
        # timeout以秒计，限制一条速度指令能维持多久，不会由网络轮询自动延长。
        if not math.isfinite(timeout) or not 0.05 <= timeout <= 0.25:
            raise ValueError('cmd_timeout 必须在 0.05..0.25 秒内')
        self.timeout = timeout
        self.disarm()

    def disarm(self):
        # 撤销许可时同时丢弃目标和接收时刻，重新arm后必须等下一条新指令。
        self.armed = False
        self._received = None
        self._velocity = (0, 0, 0)

    def arm(self):
        # 清掉使能前收到的任何目标；必须重新收到 ROS 指令才能运动。
        self.disarm()
        self.armed = True

    def accept(self, vx, vy, wz, now, *, stamp_age=0.0, frame='base_link'):
        # 先归零再校验，因此新来的无效消息不会让上一条非零目标继续保留。
        # 输入采用ROS单位：vx/vy为m/s，wz为rad/s；frame只接受车身坐标系。
        self._velocity = (0, 0, 0)
        self._received = None
        if (frame not in ('', 'base_link') or
                not all(math.isfinite(v) for v in (vx, vy, wz, now, stamp_age)) or
                abs(vx) > 0.15 or abs(vy) > 0.15 or abs(wz) > 0.6 or
                stamp_age < -0.1 or stamp_age >= self.timeout):
            return False
        if self.armed:
            # 网络/串口统一使用整数mm/s和mrad/s：0.08m/s -> 80mm/s。
            # round避免直接截断小数造成偏差；仅保存目标，不在这里发送数据。
            self._velocity = (round(vx * 1000), round(vy * 1000), round(wz * 1000))
            # 把源消息已经经历的时间计入超时，不能从收到旧包时重新续命。
            self._received = now - max(stamp_age, 0.0)
        return True

    def velocity(self, now):
        # 50ms发送周期调用此方法取缓存；没有新鲜命令就返回零速，避免持续重发旧目标。
        if (not self.armed or self._received is None or
                not 0.0 <= now - self._received < self.timeout):
            return (0, 0, 0)
        return self._velocity


def joint_values(telemetry, cpr):
    """只换算真实四轮反馈；PWM 不是扭矩，不填写 JointState.effort。"""
    # 输入顺序必须与串口协议一致：左前FL、右前FR、左后RL、右后RR。
    if len(cpr) != 4 or any(not math.isfinite(v) or v <= 0 for v in cpr):
        raise ValueError('四路 CPR 必须为正有限数')
    wheels = telemetry.get('wheels', [])
    if len(wheels) != 4:
        raise ValueError('需要 FL/FR/RL/RR 四路遥测')
    ids = ['FL', 'FR', 'RL', 'RR']
    positions, velocities = [], []
    for i, wheel in enumerate(wheels):
        if wheel.get('id', '').upper() != ids[i]:
            raise ValueError('遥测轮序错误')
        # count/CPR为输出轴累计圈数，每圈2πrad；rpm_x10先除10恢复RPM，
        # 再乘2π/60转换成ROS JointState要求的rad/s。位置不做2π取模，保留累计转动。
        positions.append(int(wheel['count']) / cpr[i] * 2 * math.pi)
        velocities.append(float(wheel['rpm_x10']) / 10 * 2 * math.pi / 60)
    return positions, velocities
