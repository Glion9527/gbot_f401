"""窗口按键状态机；不依赖 ROS/Tk，输入时钟使用 time.monotonic。

短暂延后 KeyRelease，识别 Linux 的 Release/Press 自动重复；运动本身不
依靠重复字符续期。失焦、泵卡顿、事件租期和单次操作时限均要求重新使能。
"""
import math


class HoldState:
    DIRECTIONS = frozenset('wsadqe')

    def __init__(self, linear_speed=0.08, angular_speed=0.25,
                 release_delay=0.015, event_lease=1.0,
                 pump_timeout=0.2, motion_limit=5.0):
        values = (linear_speed, angular_speed, release_delay, event_lease,
                  pump_timeout, motion_limit)
        if (not all(math.isfinite(v) and v > 0 for v in values) or
                linear_speed > 0.15 or angular_speed > 0.6):
            raise ValueError('按键速度或时间参数无效')
        self.linear_speed, self.angular_speed = linear_speed, angular_speed
        self.release_delay, self.event_lease = release_delay, event_lease
        self.pump_timeout, self.motion_limit = pump_timeout, motion_limit
        self.focused = False
        self.enabled = False
        self._held = set()
        self._quarantined = set()
        self._release_at = {}
        self._active = None
        self._await_neutral = True
        self._conflict = False
        self._last_event = None
        self._last_pump = None
        self._motion_started = None

    @staticmethod
    def _key(key):
        key = key.lower()
        return {' ': 'space', 'escape': 'space', 'kp_0': '0'}.get(key, key)

    def halt(self):
        """清运动意图，保留物理按住记录，避免旧的自动重复变成新的使能键。"""
        self.enabled = False
        self._active = None
        self._await_neutral = True
        self._conflict = False
        self._motion_started = None

    def focus(self, focused):
        self.focused = bool(focused)
        if not self.focused:
            self.halt()
            # 旧方向不阻塞重新使能，但必须真实松开才能再次运动；否则回焦后的
            # 自动重复可能被误认作新按下。控制键仍保留去重，防止旧r重复使能。
            self._quarantined.update(self._held.intersection(self.DIRECTIONS))
            self._held.intersection_update(('r', 'space'))
            # 失焦前尚未确认的方向Release可能是自动重复的前半个事件，
            # 不能用它解除隔离；只接受失焦后再次观察到的松键。
            self._release_at = {key: due for key, due in self._release_at.items()
                                if key in self._held}

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        if not enabled:
            self.halt()
        elif not self.enabled:
            self.enabled = True
            self._active = None
            self._motion_started = None
            # 使能RPC等待期间按住的键不能在成功后变成运动。
            self._await_neutral = bool(self._held)
            self._conflict = False

    def _expire_releases(self, now):
        for key, due in list(self._release_at.items()):
            if now >= due:
                del self._release_at[key]
                self._held.discard(key)
                self._quarantined.discard(key)
                if self._active == key:
                    self._active = None
        if not self._held:
            self._await_neutral = False
        if not self._held.intersection(self.DIRECTIONS):
            self._conflict = False

    def press(self, key, now):
        key = self._key(key)
        self._expire_releases(now)
        self._last_event = now
        self._release_at.pop(key, None)
        if not self.focused or key in self._held or key in self._quarantined:
            return None
        if key not in self.DIRECTIONS and key not in ('r', 'space', '0'):
            self._active = None
            self._await_neutral = True
            return 'zero'
        self._held.add(key)
        if key in ('r', 'space'):
            self.halt()
            return 'arm' if key == 'r' else 'stop'
        if key == '0':
            self._active = None
            self._await_neutral = True
            return 'zero'
        if len(self._held.intersection(self.DIRECTIONS)) > 1:
            # 不把松开其中一键解释为恢复另一个旧方向，必须先全部释放。
            self._active = None
            self._conflict = True
        elif self.enabled and not self._await_neutral and not self._conflict:
            self._active = key
            if self._motion_started is None:
                self._motion_started = now
        return None

    def release(self, key, now):
        key = self._key(key)
        self._last_event = now
        if key in self._held or key in self._quarantined:
            self._release_at[key] = now + self.release_delay

    def pump(self, now):
        """在每次进入 ROS spin/新发布前调用；返回需要解除使能的原因。"""
        gap = None if self._last_pump is None else now - self._last_pump
        self._last_pump = now
        self._expire_releases(now)
        reason = None
        if gap is not None and (gap < 0 or gap >= self.pump_timeout):
            reason = '窗口处理停顿，已解除使能'
        elif (self.enabled and self._motion_started is not None and
              now - self._motion_started >= self.motion_limit):
            reason = '本次控制已满5秒，请松开按键后重新按r'
        elif (self.enabled and self._held.intersection(self.DIRECTIONS) and
              self._last_event is not None and now - self._last_event >= self.event_lease):
            reason = '超过1秒没有新的键事件，已解除使能'
        if reason:
            self.halt()
        return reason

    def velocity(self, now):
        self._expire_releases(now)
        if (not self.focused or not self.enabled or self._await_neutral or
                self._conflict or self._active is None):
            return (0.0, 0.0, 0.0)
        speed, turn = self.linear_speed, self.angular_speed
        return {'w': (speed, 0.0, 0.0), 's': (-speed, 0.0, 0.0),
                'a': (0.0, speed, 0.0), 'd': (0.0, -speed, 0.0),
                'q': (0.0, 0.0, turn), 'e': (0.0, 0.0, -turn)}[self._active]
