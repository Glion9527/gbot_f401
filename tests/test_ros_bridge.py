"""隔离 ROS/网络的节点行为测试；不声称替代 Ubuntu 中的 rclpy 实机验收。"""
import copy
import importlib
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/f401_robot_bridge'))


class Message:
    def __init__(self):
        self.header = NS(stamp=NS(sec=100, nanosec=0), frame_id='base_link')
        self.linear, self.angular = NS(x=0., y=0.), NS(z=0.)
        self.twist = NS(linear=self.linear, angular=self.angular)


class FakeNode:
    def __init__(self, name):
        self.params = {}
        self.sent = {}
    def declare_parameter(self, name, value):
        self.params[name] = value
    def get_parameter(self, name):
        return NS(value=self.params[name])
    def create_publisher(self, cls, name, depth):
        self.sent[name] = []
        return NS(publish=lambda msg: self.sent[name].append(msg))
    def create_subscription(self, *args):
        return args
    def create_service(self, *args):
        return args
    def create_timer(self, *args):
        return args
    def create_client(self, *args):
        return NS(service_is_ready=lambda: True)
    def get_logger(self):
        return NS(info=lambda x: None, warning=lambda x: None)
    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=100000000000, to_msg=lambda: NS(sec=100, nanosec=0)))


class LinkError(Exception):
    pass


class Client:
    instances = []
    def __init__(self, *args, **kwargs):
        self.calls = []
        self.closed = False
        self.fail = False
        self.arm_confirmed = True
        self.t = dict(session=123, mcu_ms=1000, armed=False, last_seq=0,
                      fault_mask=0, age_ms=0,
                      wheels=[dict(id=i, count=0, rpm_x10=0, target_x10=0, pwm=0)
                              for i in ['FL', 'FR', 'RL', 'RR']])
        self.instances.append(self)
    def connect(self):
        return dict(telemetry=copy.deepcopy(self.t))
    def request(self, kind, **fields):
        self.calls.append((kind, fields))
        if self.fail:
            raise LinkError('simulated loss')
        self.t['mcu_ms'] += 10
        if kind == 'arm':
            self.t['armed'] = self.arm_confirmed
        if kind == 'stop':
            self.t['armed'] = False
        return dict(ok=True, reason='OK', telemetry=copy.deepcopy(self.t))
    def close(self):
        self.closed = True


def load_nodes():
    modules = {}
    for name in ['rclpy', 'rclpy.node', 'geometry_msgs', 'geometry_msgs.msg',
                 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg',
                 'std_srvs', 'std_srvs.srv', 'robot_link', 'robot_link.client']:
        modules[name] = ModuleType(name)
    modules['rclpy.node'].Node = FakeNode
    modules['geometry_msgs.msg'].Twist = Message
    modules['geometry_msgs.msg'].TwistStamped = Message
    modules['sensor_msgs.msg'].JointState = Message
    modules['std_msgs.msg'].String = Message
    modules['std_srvs.srv'].Trigger = NS(Request=lambda: NS())
    modules['robot_link.client'].RobotClient = Client
    modules['robot_link.client'].RobotLinkError = LinkError
    with patch.dict(sys.modules, modules):
        b = importlib.import_module('f401_robot_bridge.bridge_node')
        k = importlib.import_module('f401_robot_bridge.keyboard_node')
        w = importlib.import_module('f401_robot_bridge.keyboard_window')
    return b, k, w


bridge, keyboard, keyboard_window = load_nodes()


class NodeTests(unittest.TestCase):
    def make_node(self):
        with patch.object(Path, 'read_text', return_value='unit-test-placeholder'):
            return bridge.NetworkBridge()

    def test_connect_is_disarmed_and_arm_discards_old_command(self):
        node = self.make_node()
        client = node._client
        node._state.accept(.05, 0, 0, 10.)
        node._tick()
        self.assertEqual(client.calls[-1][0], 'status')
        response = node._arm_service(None, NS())
        self.assertTrue(response.success)
        with patch.object(bridge.time, 'monotonic', return_value=10.1):
            node._tick()
        self.assertEqual(client.calls[-1], ('velocity', dict(vx_mm_s=0, vy_mm_s=0, wz_mrad_s=0)))

    def test_ros_expiry_zeros_even_with_healthy_tcp(self):
        node = self.make_node()
        node._arm_service(None, NS())
        node._state.accept(.05, 0, 0, 10.)
        with patch.object(bridge.time, 'monotonic', return_value=10.1):
            node._tick()
        self.assertEqual(node._client.calls[-1][1]['vx_mm_s'], 50)
        with patch.object(bridge.time, 'monotonic', return_value=10.3):
            node._tick()
        self.assertEqual(node._client.calls[-1][1]['vx_mm_s'], 0)

    def test_stamped_message_queued_before_arm_does_not_move(self):
        node = self.make_node()
        node._arm_service(None, NS())
        message = Message()
        message.twist.linear.x = .05
        # 同一条旧消息在ARM成功后才被executor取出，也不能续成运动许可。
        with patch.object(bridge.time, 'monotonic', return_value=10.0):
            node._on_stamped(message)
        self.assertEqual(node._state.velocity(10.01), (0, 0, 0))
        message.header.stamp.nanosec = 1000000
        with patch.object(bridge.time, 'monotonic', return_value=10.0):
            node._on_stamped(message)
        self.assertEqual(node._state.velocity(10.01), (50, 0, 0))

    def test_network_loss_disarms_and_does_not_reconnect(self):
        node = self.make_node()
        node._arm_service(None, NS())
        client = node._client
        count = len(Client.instances)
        client.fail = True
        node._tick()
        self.assertFalse(node._state.armed)
        self.assertIsNone(node._client)
        self.assertTrue(client.closed)
        node._tick()
        self.assertEqual(len(Client.instances), count)

    def test_arm_ack_without_confirmed_state_is_rejected(self):
        node = self.make_node()
        client = node._client
        client.arm_confirmed = False
        response = node._arm_service(None, NS())
        self.assertFalse(response.success)
        self.assertFalse(node._state.armed)
        self.assertEqual(client.calls[-1][0], 'stop')

    def test_repeated_arm_stops_instead_of_restarting(self):
        node = self.make_node()
        node._arm_service(None, NS())
        response = node._arm_service(None, NS())
        self.assertFalse(response.success)
        self.assertFalse(node._state.armed)
        self.assertEqual(node._client.calls[-1][0], 'stop')

    def test_stale_telemetry_is_not_republished_with_new_time(self):
        node = self.make_node()
        t = copy.deepcopy(node._telemetry)
        count = len(node.sent['/joint_states'])
        node._process_telemetry(t)
        self.assertEqual(len(node.sent['/joint_states']), count)
        t['age_ms'] = 500
        with self.assertRaises(ValueError):
            node._process_telemetry(t)

    def test_stop_wins_pending_keyboard_arm(self):
        futures = []
        class Future:
            def done(self):
                return False
            def add_done_callback(self, callback):
                self.callback = callback
            def result(self):
                return NS(success=True, message='OK')
        def call(request):
            future = Future()
            futures.append(future)
            return future
        node = keyboard.Keyboard()
        node._arm.call_async = node._stop.call_async = call
        node._service(True)
        node._service(False)
        self.assertEqual(len(futures), 2)
        futures[0].callback(futures[0])
        self.assertFalse(node._enabled)
        futures[1].callback(futures[1])
        self.assertFalse(node._enabled)

    def test_fault_is_visible_even_when_status_request_succeeds(self):
        node = self.make_node()
        node._client.t['fault_mask'] = 1
        node._tick()
        self.assertIn('fault_mask=1', node._reason)
        self.assertIn('无编码器反馈', node.sent['/robot/state'][-1].data)
        self.assertFalse(node._state.armed)

    def test_terminal_keyboard_uses_verified_speed_and_keeps_timeout(self):
        node = keyboard.Keyboard()
        node._enabled = True
        with patch.object(keyboard.select, 'select', return_value=([None], [], [])), \
             patch.object(keyboard.os, 'read', return_value=b'w'), \
             patch.object(keyboard.time, 'monotonic', return_value=1.0):
            node._tick()
        self.assertEqual(node.sent['/cmd_vel'][-1].twist.linear.x, .08)
        with patch.object(keyboard.select, 'select', return_value=([], [], [])), \
             patch.object(keyboard.time, 'monotonic', return_value=1.3):
            node._tick()
        self.assertEqual(node.sent['/cmd_vel'][-1].twist.linear.x, 0.0)


class WindowTests(unittest.TestCase):
    """使用真实窗口节点方法和假RPC，覆盖输入状态与异步服务的衔接。"""
    def make_node(self):
        class Future:
            def __init__(self):
                self.callbacks = []
                self.finished = False
            def done(self):
                return self.finished
            def add_done_callback(self, callback):
                self.callbacks.append(callback)
            def result(self):
                return NS(success=True, message='OK')
            def complete(self):
                self.finished = True
                for callback in self.callbacks:
                    callback(self)
        node = keyboard_window.WindowKeyboard()
        futures = []
        def call(request):
            future = Future()
            futures.append(future)
            return future
        node._arm.call_async = node._stop.call_async = call
        node.set_focus(True)
        return node, futures

    def test_pending_arm_cannot_resume_after_focus_loss(self):
        node, futures = self.make_node()
        node.key_press('r')
        node.set_focus(False)
        self.assertEqual(len(futures), 2)
        futures[0].complete()
        node._tick()
        self.assertFalse(node._enabled)
        self.assertEqual(node.sent['/cmd_vel'][-1].twist.linear.x, 0.0)
        futures[1].complete()
        node.set_focus(True)
        node.key_press('w')
        node._tick()
        self.assertFalse(node._enabled)
        self.assertEqual(node.sent['/cmd_vel'][-1].twist.linear.x, 0.0)

    def test_hold_and_release_reach_stamped_publisher(self):
        node, futures = self.make_node()
        with patch.object(keyboard_window.time, 'monotonic', return_value=0.0):
            node.key_press('r')
            node.key_release('r')
        futures[0].complete()
        with patch.object(keyboard_window.time, 'monotonic', return_value=.02):
            node.before_spin()
            node._tick()
            node.key_press('w')
        # 模拟每10ms泵运行，首次OS自动重复前仍持续发送速度。
        for index in range(3, 71):
            with patch.object(keyboard_window.time, 'monotonic', return_value=index / 100):
                node.before_spin()
                node._tick()
        self.assertEqual(node.sent['/cmd_vel'][-1].twist.linear.x, .08)
        with patch.object(keyboard_window.time, 'monotonic', return_value=.71):
            node.key_release('w')
        with patch.object(keyboard_window.time, 'monotonic', return_value=.73):
            node.before_spin()
            node._tick()
        self.assertEqual(node.sent['/cmd_vel'][-1].twist.linear.x, 0.0)

    def test_event_pump_gap_cancels_pending_arm_before_publish(self):
        node, futures = self.make_node()
        with patch.object(keyboard_window.time, 'monotonic', return_value=0.0):
            node.key_press('r')
            node.before_spin()
        with patch.object(keyboard_window.time, 'monotonic', return_value=.3):
            node.before_spin()
        self.assertEqual(len(futures), 2)
        futures[0].complete()
        node._tick()
        self.assertFalse(node._enabled)
        self.assertIn('停顿', node.notice)
        self.assertEqual(node.sent['/cmd_vel'][-1].twist.linear.x, 0.0)


if __name__ == '__main__':
    unittest.main()
