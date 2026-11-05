"""窗口输入规则的假时钟测试；不启动Tk、ROS、网络或真实串口。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/f401_robot_bridge'))
from f401_robot_bridge.hold_state import HoldState


class HoldTests(unittest.TestCase):
    def armed(self):
        state = HoldState()
        state.focus(True)
        state.set_enabled(True)
        state.pump(0.)
        return state

    def pump_until(self, state, start, end):
        now = start
        while now < end:
            now = min(now + .01, end)
            result = state.pump(now)
            if result:
                return result
        return None

    def test_press_hold_without_first_repeat_pause(self):
        state = self.armed()
        state.press('w', .01)
        self.assertEqual(state.velocity(.02), (.08, 0., 0.))
        self.assertIsNone(self.pump_until(state, .01, .7))
        self.assertEqual(state.velocity(.7), (.08, 0., 0.))

    def test_true_release_zeroes_after_short_filter(self):
        state = self.armed()
        state.press('w', .01)
        state.release('w', .1)
        self.assertEqual(state.velocity(.11), (.08, 0., 0.))
        self.assertEqual(state.velocity(.116), (0., 0., 0.))

    def test_autorepeat_release_press_does_not_pause_or_rearm(self):
        state = self.armed()
        state.press('w', .01)
        state.release('w', .5)
        self.assertIsNone(state.press('w', .501))
        self.assertEqual(state.velocity(.52), (.08, 0., 0.))
        state.halt()
        self.assertEqual(state.press('r', .6), 'arm')
        state.release('r', .7)
        self.assertIsNone(state.press('r', .701))
        self.assertIsNone(state.press('r', .8))

    def test_arm_pending_keys_need_release_and_new_press(self):
        state = HoldState()
        state.focus(True)
        self.assertEqual(state.press('r', 0.), 'arm')
        state.press('w', .01)
        state.set_enabled(True)
        self.assertEqual(state.velocity(.02), (0., 0., 0.))
        state.release('r', .03)
        state.release('w', .03)
        self.assertEqual(state.velocity(.05), (0., 0., 0.))
        state.press('w', .06)
        self.assertEqual(state.velocity(.07), (.08, 0., 0.))

    def test_conflicting_directions_do_not_resume_old_direction(self):
        state = self.armed()
        state.press('w', .01)
        state.press('s', .02)
        self.assertEqual(state.velocity(.03), (0., 0., 0.))
        state.release('s', .04)
        self.assertEqual(state.velocity(.06), (0., 0., 0.))
        state.release('w', .07)
        state.velocity(.09)
        state.press('a', .1)
        self.assertEqual(state.velocity(.11), (0., .08, 0.))

    def test_focus_loss_and_stop_do_not_auto_resume(self):
        state = self.armed()
        state.press('w', .01)
        state.focus(False)
        state.focus(True)
        self.assertIsNone(state.press('w', .02))
        self.assertEqual(state.velocity(.03), (0., 0., 0.))
        self.assertEqual(state.press('space', .04), 'stop')
        self.assertIsNone(state.press('space', .05))

    def test_focus_missed_direction_release_needs_one_tap_to_recover(self):
        state = self.armed()
        state.press('w', .01)
        state.focus(False)
        # 模拟在别的窗口松开w，当前窗口收不到KeyRelease。
        state.focus(True)
        self.assertEqual(state.press('r', .02), 'arm')
        state.release('r', .03)
        state.velocity(.05)
        state.set_enabled(True)
        state.press('w', .06)
        self.assertEqual(state.velocity(.07), (0., 0., 0.))
        # 在窗口外漏掉的松键无法证明；窗口内点放一次，下一次按才允许运动。
        state.release('w', .08)
        state.press('w', .1)
        self.assertEqual(state.velocity(.11), (.08, 0., 0.))

    def test_focus_old_direction_repeat_cannot_move_after_rearm(self):
        state = self.armed()
        state.press('w', .01)
        state.focus(False)
        state.focus(True)
        self.assertEqual(state.press('r', .02), 'arm')
        state.release('r', .03)
        state.velocity(.05)
        state.set_enabled(True)
        # Linux重复事件的Release+Press对不能解除失焦隔离。
        state.release('w', .06)
        state.press('w', .061)
        self.assertEqual(state.velocity(.08), (0., 0., 0.))
        state.press('w', .1)
        self.assertEqual(state.velocity(.11), (0., 0., 0.))

    def test_focus_preserves_control_edge_until_true_release(self):
        state = HoldState()
        state.focus(True)
        self.assertEqual(state.press('r', 0.), 'arm')
        state.focus(False)
        state.focus(True)
        self.assertIsNone(state.press('r', .01))
        state.release('r', .02)
        self.assertEqual(state.press('r', .04), 'arm')

    def test_unconfirmed_repeat_release_before_focus_loss_stays_quarantined(self):
        state = self.armed()
        state.press('w', .01)
        state.release('w', .02)
        state.focus(False)
        # 假Release的配对Press因失焦没有送达，旧Release不可解除隔离。
        state.focus(True)
        self.assertEqual(state.press('r', .04), 'arm')
        state.release('r', .05)
        state.velocity(.07)
        state.set_enabled(True)
        state.press('w', .08)
        self.assertEqual(state.velocity(.09), (0., 0., 0.))

    def test_gui_stall_stops_before_resume(self):
        state = self.armed()
        state.press('w', .01)
        self.assertIn('停顿', state.pump(.3))
        self.assertFalse(state.enabled)
        self.assertEqual(state.velocity(.3), (0., 0., 0.))
        self.assertIsNone(state.press('w', .31))

    def test_missing_key_events_expires_lease(self):
        state = self.armed()
        state.press('w', .01)
        self.assertIn('键事件', self.pump_until(state, 0., 1.02))
        self.assertFalse(state.enabled)
        self.assertEqual(state.velocity(1.03), (0., 0., 0.))

    def test_repeat_events_cannot_extend_five_second_limit(self):
        state = self.armed()
        state.press('w', .01)
        now = 0.
        reason = None
        while now < 5.1:
            now = round(now + .01, 4)
            state.press('w', now)
            reason = state.pump(now)
            if reason:
                break
        self.assertIn('5秒', reason)
        self.assertLessEqual(now, 5.02)
        self.assertFalse(state.enabled)
        self.assertIsNone(state.press('w', now + .01))
        self.assertEqual(state.velocity(now + .02), (0., 0., 0.))

    def test_zero_and_direction_change_do_not_reset_absolute_limit(self):
        state = self.armed()
        previous = 0.
        for now, operation, key in [(.01, 'press', 'w'), (.1, 'release', 'w'),
                                    (.2, 'press', '0'), (.3, 'release', '0'),
                                    (.4, 'press', 'a')]:
            self.assertIsNone(self.pump_until(state, previous, now))
            getattr(state, operation)(key, now)
            previous = now
        self.assertEqual(state.velocity(.41), (0., .08, 0.))
        now = .4
        reason = None
        while now < 5.02:
            now = round(now + .01, 4)
            state.press('a', now)
            reason = state.pump(now)
            if reason:
                break
        self.assertIn('5秒', reason)
        self.assertFalse(state.enabled)


if __name__ == '__main__':
    unittest.main()
