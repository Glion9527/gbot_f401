"""不依赖 ROS 安装的指令时效/轮速单位测试。"""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/f401_robot_bridge'))
from f401_robot_bridge.control_state import CommandState, joint_values


# 直接测试不依赖 rclpy 的状态类，重点核对 ROS 浮点单位到串口整数单位的关系。
class CommandTests(unittest.TestCase):
    def test_no_implicit_arm_or_old_velocity(self):
        s = CommandState()
        s.accept(.05, 0, 0, 10.0)
        self.assertEqual(s.velocity(10.0), (0, 0, 0))
        s.arm()
        self.assertEqual(s.velocity(10.0), (0, 0, 0))
        s.accept(.05, -.05, .25, 10.0)
        self.assertEqual(s.velocity(10.2), (50, -50, 250))
        self.assertEqual(s.velocity(10.25), (0, 0, 0))
        self.assertEqual(s.velocity(9.0), (0, 0, 0))
        s.disarm()
        s.arm()
        self.assertEqual(s.velocity(10.1), (0, 0, 0))

    def test_source_age_counts_toward_expiry(self):
        # 收到前已经耗掉的消息时间也计入有效期，不能把旧目标当成刚发布的新目标。
        s = CommandState()
        s.arm()
        self.assertTrue(s.accept(.1, 0, 0, 10, stamp_age=.2))
        self.assertEqual(s.velocity(10.01), (100, 0, 0))
        self.assertEqual(s.velocity(10.06), (0, 0, 0))

    def test_invalid_target_clears_previous(self):
        for kwargs in [dict(vx=math.nan), dict(vx=math.inf), dict(vx=.301),
                       dict(vy=-.151), dict(wz=.601), dict(stamp_age=.25),
                       dict(stamp_age=-.101), dict(frame='map')]:
            with self.subTest(kwargs=kwargs):
                s = CommandState()
                s.arm()
                s.accept(.05, 0, 0, 1.0)
                args = dict(vx=0., vy=0., wz=0., now=1.)
                args.update(kwargs)
                self.assertFalse(s.accept(**args))
                self.assertEqual(s.velocity(1.1), (0, 0, 0))

    def test_joint_unit_conversion_and_order(self):
        # 用已知整圈计数验证弧度换算，同时确认轮序必须是 FL/FR/RL/RR。
        t = {'wheels': [dict(id=i, count=3960 * k, rpm_x10=150)
                        for k, i in enumerate(['FL', 'FR', 'RL', 'RR'])]}
        position, velocity = joint_values(t, [3960.] * 4)
        self.assertAlmostEqual(position[3], 6 * math.pi)
        self.assertAlmostEqual(velocity[0], math.pi / 2)
        t['wheels'][0]['id'] = 'FR'
        with self.assertRaises(ValueError):
            joint_values(t, [3960.] * 4)


if __name__ == '__main__':
    unittest.main()
