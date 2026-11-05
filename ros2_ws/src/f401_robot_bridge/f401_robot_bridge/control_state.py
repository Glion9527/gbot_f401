"""与 ROS 分离的运动许可：只保存新鲜指令，连接/使能都不复用旧目标。"""
import math


def fault_description(mask):
    """硬件故障与网络请求成功是两个状态，避免状态查询OK掩盖故障。"""
    names = [(1, '200ms无编码器反馈'), (2, '反馈方向相反'),
             (4, '电机看门狗超时'), (8, '参数配置错误'), (16, '外设启动失败')]
    reasons = [name for bit, name in names if mask & bit]
    if mask & ~31:
        reasons.append('未知故障位')
    return 'fault_mask=%d（%s）' % (mask, '、'.join(reasons))


class CommandState:
    def __init__(self, timeout=0.25):
        if not math.isfinite(timeout) or not 0.05 <= timeout <= 0.25:
            raise ValueError('cmd_timeout 必须在 0.05..0.25 秒内')
        self.timeout = timeout
        self.disarm()

    def disarm(self):
        self.armed = False
        self._received = None
        self._velocity = (0, 0, 0)

    def arm(self):
        # 清掉使能前收到的任何目标；必须重新收到 ROS 指令才能运动。
        self.disarm()
        self.armed = True

    def accept(self, vx, vy, wz, now, *, stamp_age=0.0, frame='base_link'):
        self._velocity = (0, 0, 0)
        self._received = None
        if (frame not in ('', 'base_link') or
                not all(math.isfinite(v) for v in (vx, vy, wz, now, stamp_age)) or
                abs(vx) > 0.15 or abs(vy) > 0.15 or abs(wz) > 0.6 or
                stamp_age < -0.1 or stamp_age >= self.timeout):
            return False
        if self.armed:
            self._velocity = (round(vx * 1000), round(vy * 1000), round(wz * 1000))
            # 把源消息已经经历的时间计入超时，不能从收到旧包时重新续命。
            self._received = now - max(stamp_age, 0.0)
        return True

    def velocity(self, now):
        if (not self.armed or self._received is None or
                not 0.0 <= now - self._received < self.timeout):
            return (0, 0, 0)
        return self._velocity


def joint_values(telemetry, cpr):
    """只换算真实四轮反馈；PWM 不是扭矩，不填写 JointState.effort。"""
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
        positions.append(int(wheel['count']) / cpr[i] * 2 * math.pi)
        velocities.append(float(wheel['rpm_x10']) / 10 * 2 * math.pi / 60)
    return positions, velocities
