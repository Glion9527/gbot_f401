"""窗口输入规则的假时钟测试；不启动Tk、ROS、网络或真实串口。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/f401_robot_bridge'))
from f401_robot_bridge.hold_state import HoldState


# 各测试传入明确的时间数值，模拟真实松键、系统重复按键和窗口事件停顿。
# 这种“假时钟”让时间边界能稳定复现，不需要真的等待几秒或打开控制窗口。
class HoldTests(unittest.TestCase):
    def armed(self):
        state = HoldState()
        state.focus(True)
        state.set_enabled(True)
        state.pump(0.)
        return state

    def pump_until(self, state, start, end):
        # 模拟 GUI 定期处理事件；一旦状态机要求停止，就立即返回原因供测试断言。
        now = start
        while now < end:
            now = min(now + .01, end)
            result = state.pump(now)
            if result:
                return result
        return None

    def tap(self, state, key, now):
        state.press(key, now)
        state.release(key, now + .005)
        return state.velocity(now + .025)

    def test_tap_latches_velocity_after_release_without_key_events(self):
        state = self.armed()
        self.assertEqual(self.tap(state, 'w', .01), (.08, 0., 0.))
        self.assertIsNone(self.pump_until(state, .04, 1.5))
        self.assertEqual(state.velocity(1.5), (.08, 0., 0.))

    def test_each_confirmed_press_changes_one_step(self):
        state = self.armed()
        state.press('w', .01)
        state.press('w', .09)
        state.release('w', .1)
        state.press('w', .101)
        self.assertEqual(state.velocity(.12), (.08, 0., 0.))
        state.release('w', .13)
        self.assertEqual(self.tap(state, 'w', .16), (.10, 0., 0.))

    def test_autorepeat_cannot_rearm(self):
        state = self.armed()
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

    def test_signed_steps_clamp_and_pass_through_zero(self):
        for axis, (positive, negative, steps) in enumerate((
            ('w', 'x', (.08, .10, .12, .14, .15)),
            ('a', 'd', (.08, .10, .12, .14, .15)),
            ('q', 'e', (.25, .35, .45, .55, .60)),
        )):
            with self.subTest(axis=axis):
                state = self.armed()
                presses = [
                    *((positive, speed) for speed in (*steps, steps[-1])),
                    *((negative, speed) for speed in reversed(steps[:-1])),
                    (negative, 0.),
                    *((negative, -speed) for speed in (*steps, steps[-1])),
                    (positive, -steps[-2]),
                ]
                for index, (key, expected) in enumerate(presses, 1):
                    velocity = self.tap(state, key, index * .04)
                    self.assertAlmostEqual(velocity[axis], expected)
                    self.assertTrue(all(value == 0. for other, value in enumerate(velocity)
                                        if other != axis))

    def test_mixed_axes_and_zero_keys_stay_armed(self):
        state = self.armed()
        self.assertEqual(self.tap(state, 'w', .01), (.08, 0., 0.))
        self.assertEqual(self.tap(state, 'a', .05), (.08, .08, 0.))
        self.assertEqual(self.tap(state, 'e', .09), (.08, .08, -.25))
        self.assertEqual(self.tap(state, 's', .13), (0., 0., 0.))
        self.assertTrue(state.enabled)
        self.assertEqual(self.tap(state, 'x', .17), (-.08, 0., 0.))
        self.assertEqual(self.tap(state, '0', .21), (0., 0., 0.))
        self.assertTrue(state.enabled)

    def test_stop_and_rearm_discard_latched_target(self):
        state = self.armed()
        self.assertEqual(self.tap(state, 'w', .01), (.08, 0., 0.))
        self.assertEqual(state.press('space', .05), 'stop')
        self.assertFalse(state.enabled)
        self.assertEqual(state.velocity(.06), (0., 0., 0.))
        state.release('space', .07)
        self.assertEqual(state.press('r', .10), 'arm')
        state.release('r', .11)
        state.set_enabled(True)
        self.assertEqual(state.velocity(.13), (0., 0., 0.))
        self.assertEqual(self.tap(state, 'w', .15), (.08, 0., 0.))

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

    def test_repeat_events_do_not_change_latched_speed_or_prevent_stop(self):
        state = self.armed()
        state.press('w', .01)
        for index in range(1, 611):
            now = index / 100
            state.press('w', now)
            self.assertIsNone(state.pump(now))
        self.assertTrue(state.enabled)
        self.assertEqual(state.velocity(6.10), (.08, 0., 0.))
        self.assertEqual(state.press('space', 6.11), 'stop')
        self.assertFalse(state.enabled)
        self.assertEqual(state.velocity(6.12), (0., 0., 0.))

    def test_zero_and_direction_change_work_after_extended_motion(self):
        state = self.armed()
        self.assertEqual(self.tap(state, 'w', .01), (.08, 0., 0.))
        self.assertIsNone(self.pump_until(state, .04, 6.0))
        self.assertEqual(state.velocity(6.0), (.08, 0., 0.))
        self.assertEqual(self.tap(state, '0', 6.01), (0., 0., 0.))
        self.assertTrue(state.enabled)
        self.assertEqual(self.tap(state, 'a', 6.05), (0., .08, 0.))
        self.assertIsNone(self.pump_until(state, 6.08, 12.0))
        self.assertTrue(state.enabled)
        self.assertEqual(state.velocity(12.0), (0., .08, 0.))


if __name__ == '__main__':
    unittest.main()
