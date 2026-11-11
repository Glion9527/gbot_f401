"""终端和窗口共用的速度档位，以及窗口独有的按键状态。"""
import math


class SpeedGears:
    # 下标为前后、横移、转向；正值分别对应向前、向左、逆时针。
    DIRECTIONS = {'w': (0, 1), 'x': (0, -1), 'a': (1, 1), 'd': (1, -1),
                  'q': (2, 1), 'e': (2, -1)}

    def __init__(self, linear_speed, angular_speed):
        if not (0 < linear_speed <= 0.30 and 0 < angular_speed <= 0.60):
            raise ValueError('linear_speed需在(0,.3]，angular_speed在(0,.6]')
        if linear_speed > 0.15:
            # 大于原上限时按“最高档”理解：从已验证的 0.08 起步，每档 0.02m/s
            # 升到该值；横移仍按 0.08~0.15 的五档。
            forward = self._ladder(0.08, 0.02, linear_speed)
            strafe = self._ladder(0.08, 0.02, 0.15)
        else:
            # 保持原语义：linear_speed 就是起步档，档位按 0.02m/s 递增到 0.15。
            forward = strafe = self._ladder(linear_speed, 0.02, 0.15)
        self._speeds = (forward, strafe, self._ladder(angular_speed, 0.10, 0.60))
        self.clear()

    @staticmethod
    def _ladder(start, step, limit):
        # 第一档沿用已验证的起步速度，之后按 step 递增，末尾不足一步时补上限本身。
        levels = [0.0]
        index = 0
        while True:
            speed = min(limit, round(start + step * index, 10))
            if speed <= levels[-1]:
                break
            levels.append(speed)
            index += 1
        return tuple(levels)

    def clear(self):
        self._gear = [0, 0, 0]

    def step(self, key):
        axis, change = self.DIRECTIONS[key]
        max_gear = len(self._speeds[axis]) - 1
        self._gear[axis] = max(-max_gear, min(max_gear, self._gear[axis] + change))

    def velocity(self):
        return tuple(-speeds[-gear] if gear < 0 else speeds[gear]
                     for speeds, gear in zip(self._speeds, self._gear))


class HoldState:
    def __init__(self, linear_speed=0.08, angular_speed=0.25,
                 release_delay=0.015, pump_timeout=0.2):
        if not all(math.isfinite(v) and v > 0 for v in (release_delay, pump_timeout)):
            raise ValueError('按键时间参数无效')
        self._gears = SpeedGears(linear_speed, angular_speed)
        self.release_delay = release_delay
        self.pump_timeout = pump_timeout
        self.focused = False
        self.enabled = False
        # 隔离失焦前按住的方向键，直到观察到真实松开。
        self._held = set()
        self._quarantined = set()
        self._release_at = {}
        self._await_neutral = True
        self._last_pump = None

    @staticmethod
    def _key(key):
        # Tk传入的是keysym；统一大小写以及空格/Escape/小键盘0等同义键。
        key = key.lower()
        return {' ': 'space', 'escape': 'space', 'kp_0': '0'}.get(key, key)

    def halt(self):
        """清运动意图，保留物理按住记录，避免旧的自动重复变成新的使能键。"""
        self.enabled = False
        self._gears.clear()
        self._await_neutral = True

    def focus(self, focused):
        # 窗口失焦意味着之后可能收不到松键事件，所以同时撤销运动并隔离旧方向。
        self.focused = bool(focused)
        if not self.focused:
            self.halt()
            # 旧方向不阻塞重新使能，但必须真实松开才能再次运动；否则回焦后的
            # 自动重复可能被误认作新按下。控制键仍保留去重，防止旧r重复使能。
            self._quarantined.update(self._held.intersection(SpeedGears.DIRECTIONS))
            self._held.intersection_update(('r', 'space'))
            # 失焦前尚未确认的方向Release可能是自动重复的前半个事件，
            # 不能用它解除隔离；只接受失焦后再次观察到的松键。
            self._release_at = {key: due for key, due in self._release_at.items()
                                if key in self._held}

    def set_enabled(self, enabled):
        # 与异步ROS使能回复同步。重复True不清当前按键，首次False->True才重建边界。
        enabled = bool(enabled)
        if not enabled:
            self.halt()
        elif not self.enabled:
            self.enabled = True
            # 使能RPC等待期间按住的键不能在成功后变成运动。
            self._await_neutral = bool(self._held)

    def _expire_releases(self, now):
        # 到期的松键才从_held移除。若到期前收到同键Press，press会取消待确认记录，
        # 将Linux自动重复产生的Release/Press对视作仍在按住，而不是一次新操作。
        for key, due in list(self._release_at.items()):
            if now >= due:
                del self._release_at[key]
                self._held.discard(key)
                self._quarantined.discard(key)
        if not self._held:
            self._await_neutral = False

    def press(self, key, now):
        # 返回arm/stop/zero供窗口层处理，调档则由下一次周期发布。
        key = self._key(key)
        self._expire_releases(now)
        self._release_at.pop(key, None)
        if not self.focused or key in self._held or key in self._quarantined:
            return None
        if key not in SpeedGears.DIRECTIONS and key not in ('r', 'space', 's', '0'):
            return None
        self._held.add(key)
        if key in ('r', 'space'):
            self.halt()
            return 'arm' if key == 'r' else 'stop'
        if key in ('s', '0'):
            self._gears.clear()
            self._await_neutral = True
            return 'zero'
        if self.enabled and not self._await_neutral:
            self._gears.step(key)
        return None

    def release(self, key, now):
        # 松键只允许下一次点按，已经设定的档位继续保持。
        key = self._key(key)
        if key in self._held or key in self._quarantined:
            self._release_at[key] = now + self.release_delay

    def pump(self, now):
        """在每次进入 ROS spin/新发布前调用；返回需要解除使能的原因。"""
        gap = None if self._last_pump is None else now - self._last_pump
        self._last_pump = now
        self._expire_releases(now)
        if gap is not None and (gap < 0 or gap >= self.pump_timeout):
            self.halt()
            return '窗口处理停顿，已解除使能'
        return None

    def velocity(self, now):
        self._expire_releases(now)
        if not self.focused or not self.enabled or self._await_neutral:
            return (0.0, 0.0, 0.0)
        return self._gears.velocity()
