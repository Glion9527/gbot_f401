"""终端点按遥控：200ms 没有新按键即发布零速度；终端没有 key-up 事件。"""
import os
import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


class Keyboard(Node):
    def __init__(self):
        super().__init__('f401_keyboard')
        # .08m/s 是当前底盘已验证能启动的初测速度，角速度单位为 rad/s。
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.25)
        self.declare_parameter('key_timeout', 0.2)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._key_timeout = float(self.get_parameter('key_timeout').value)
        if not (math.isfinite(self._linear_speed) and 0 < self._linear_speed <= 0.15 and
                math.isfinite(self._angular_speed) and 0 < self._angular_speed <= 0.6 and
                math.isfinite(self._key_timeout) and 0.05 <= self._key_timeout <= 0.25):
            raise ValueError('linear_speed需在(0,.15]，angular_speed在(0,.6]，key_timeout在[.05,.25]')
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self._arm = self.create_client(Trigger, '/robot/arm')
        self._stop = self.create_client(Trigger, '/robot/disarm')
        self._velocity = (0.0, 0.0, 0.0)
        self._key_at = 0.0
        self._enabled = False
        self._pending = None
        self._generation = 0
        self.create_timer(0.05, self._tick)
        self._print_controls()

    def _print_controls(self):
        print('r=显式使能；W/S=前后 A/D=横移 Q/E=左右转；空格=停止并解除使能；Ctrl-C退出。', flush=True)
        print('平移 %.3fm/s，转向 %.3frad/s；%.0fms无新按键即归零，长按受系统重复延迟影响。' % (
            self._linear_speed, self._angular_speed, self._key_timeout * 1000), flush=True)

    def _service(self, arm):
        self._enabled = False
        self._velocity = (0.0, 0.0, 0.0)
        if arm and self._pending is not None and not self._pending.done():
            return
        # 停止必须能打断尚未完成的使能请求，旧的异步结果不得重新使能键盘。
        self._generation += 1
        generation = self._generation
        client = self._arm if arm else self._stop
        if not client.service_is_ready():
            print('服务未就绪，先启动 network_bridge。', flush=True)
            return
        self._pending = client.call_async(Trigger.Request())

        def completed(future):
            if generation != self._generation:
                return
            try:
                result = future.result()
                self._enabled = bool(arm and result.success)
                self._velocity = (0.0, 0.0, 0.0)
                print(result.message, flush=True)
            except Exception as exc:
                self._enabled = False
                print('控制服务失败：' + str(exc), flush=True)
        self._pending.add_done_callback(completed)

    def _tick(self):
        # 每周期只保留最后一个键，丢弃终端排队的旧动作。
        keys = ''
        if select.select([sys.stdin], [], [], 0)[0]:
            keys = os.read(sys.stdin.fileno(), 256).decode('ascii', errors='ignore').lower()
        key = '\x03' if '\x03' in keys else ' ' if ' ' in keys else keys[-1:] or None
        if key == '\x03':
            raise KeyboardInterrupt
        if key == 'r':
            self._service(True)
        elif key == ' ':
            self._service(False)
        elif key is not None:
            speed, turn = self._linear_speed, self._angular_speed
            choices = {'w': (speed, 0.0, 0.0), 's': (-speed, 0.0, 0.0),
                       'a': (0.0, speed, 0.0), 'd': (0.0, -speed, 0.0),
                       'q': (0.0, 0.0, turn), 'e': (0.0, 0.0, -turn)}
            self._velocity = choices.get(key, (0.0, 0.0, 0.0))
            self._key_at = time.monotonic()
        velocity = self._velocity if self._enabled and time.monotonic() - self._key_at < self._key_timeout else (0.0, 0.0, 0.0)
        self.publish(velocity)

    def publish(self, velocity):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = velocity
        self._pub.publish(msg)

    def stop_before_exit(self):
        self._generation += 1
        self._enabled = False
        self.publish((0.0, 0.0, 0.0))
        if rclpy.ok() and self._stop.service_is_ready():
            future = self._stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('请在交互式 Ubuntu 终端运行键盘节点。')
    saved = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        node = Keyboard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        if node:
            if rclpy.ok():
                node.stop_before_exit()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
