"""终端点按调档。按键改变目标，定时器持续发布当前速度。"""
# 本节点把键盘操作变成/cmd_vel消息，由network_bridge转交给Mac网关。
# 它不打开串口，也不执行电机PI；使能/停止通过ROS Trigger服务请求桥接节点完成。
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

from .hold_state import SpeedGears


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('f401_keyboard')
        # .08m/s 是当前底盘已验证能启动的初测速度，角速度单位为 rad/s。
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.25)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        # 服务客户端发送一次性操作请求；持续速度通过话题发布，两者职责不同。
        self._arm = self.create_client(Trigger, '/robot/arm')
        self._stop = self.create_client(Trigger, '/robot/disarm')
        self._enabled = False
        # _pending保存未完成的服务Future；_generation标识最新请求，淘汰晚到的旧回复。
        self._pending = None
        self._generation = 0
        self.create_timer(0.05, self._tick)

    def _service(self, arm):
        # 服务确认前停止发布非零目标。
        self._enabled = False
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
            # Future可能在STOP之后才返回；只有当前代数的回复可以改变输入许可。
            if generation != self._generation:
                return
            try:
                result = future.result()
                self._enabled = bool(arm and result.success)
                print(result.message, flush=True)
            except Exception as exc:
                self._enabled = False
                print('控制服务失败：' + str(exc), flush=True)
        self._pending.add_done_callback(completed)

    def publish(self, velocity):
        # velocity的单位就是ROS标准的m/s、rad/s，这里直接填字段。
        # 时间戳供桥接节点判断消息年龄；base_link表示速度相对于车身，而非地图坐标系。
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = velocity
        self._pub.publish(msg)

    def stop_before_exit(self):
        # 使旧服务回调失效，发布零速并短暂等待解除使能服务；不无限阻塞退出。
        self._generation += 1
        self._enabled = False
        self.publish((0.0, 0.0, 0.0))
        if rclpy.ok() and self._stop.service_is_ready():
            future = self._stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)


class TerminalInput:
    """跨次读取识别终端焦点、功能键和带边界的粘贴。"""

    def __init__(self):
        self._state = 'text'
        self._csi = bytearray()
        self._pasting = False

    def feed(self, data):
        keys = []
        for byte in data:
            state = self._state
            if state == 'string' or state == 'string_escape':
                if byte == 7 or (state == 'string_escape' and byte == 92):
                    self._state = 'text'
                else:
                    self._state = 'string_escape' if byte == 27 else 'string'
            elif byte == 27:
                self._state = 'escape'
            elif state == 'escape':
                if byte == ord('['):
                    self._csi.clear()
                    self._state = 'csi'
                elif byte == ord('O') or 0x20 <= byte <= 0x2f:
                    self._state = 'function'
                elif byte in b']P^_X':
                    self._state = 'string'
                else:
                    self._state = 'text'
                    # 单独按过 Esc 后，仍要响应紧随其后的停车键。
                    if byte in b'sS0 \x03' and not self._pasting:
                        keys.append(chr(byte).lower())
            elif state in ('csi', 'function'):
                if 0x40 <= byte <= 0x7e:
                    self._state = 'text'
                    if state == 'csi':
                        sequence = bytes(self._csi) + bytes([byte])
                        if sequence in (b'I', b'O'):
                            keys.append('focus_in' if sequence == b'I' else 'focus_out')
                        elif sequence in (b'200~', b'201~'):
                            self._pasting = sequence == b'200~'
                elif state == 'csi' and len(self._csi) < 5:
                    self._csi.append(byte)
            elif byte < 128 and not self._pasting:
                keys.append(chr(byte).lower())
        return keys


class Keyboard(KeyboardNode):
    def __init__(self):
        super().__init__()
        self._gears = SpeedGears(self._linear_speed, self._angular_speed)
        self._terminal = TerminalInput()
        self._focused = True
        self._last_tick = None
        self._last_log = None
        print('点按调档 W/X 前后 · A/D 横移 · Q/E 转向 · S 停车', flush=True)
        print('R 使能 · 空格解除使能 · Ctrl-C 退出。松键保持档位，长按随终端重复输入调档。', flush=True)

    def _service(self, arm):
        self._gears.clear()
        super()._service(arm)

    def _tick(self):
        now = time.monotonic()
        gap = None if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        if gap is not None and (gap < 0 or gap >= 0.25):
            # 恢复运行时先撤销原档位，也丢弃暂停期间排队的输入。
            self._service(False)
            self._publish_target()
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
            self._terminal = TerminalInput()
            print('终端处理停顿，已解除使能。', flush=True)
            return

        keys = []
        if select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(sys.stdin.fileno(), 256)
            if not data:
                raise EOFError
            keys = self._terminal.feed(data)
        if '\x03' in keys:
            raise KeyboardInterrupt
        for key in keys:
            if key in ('focus_in', 'focus_out'):
                self._focused = key == 'focus_in'

        # 同批出现停止键时，丢弃这批里的调档和使能字符。
        if ' ' in keys or 'focus_out' in keys:
            self._service(False)
        elif 's' in keys or '0' in keys:
            self._gears.clear()
        elif self._focused and 'r' in keys:
            self._service(True)
        elif self._focused and self._enabled:
            for key in keys:
                if key in SpeedGears.DIRECTIONS:
                    self._gears.step(key)
        self._publish_target()

    def _publish_target(self):
        velocity = self._gears.velocity() if self._enabled else (0.0, 0.0, 0.0)
        self.publish(velocity)
        current = (self._enabled, velocity)
        if current != self._last_log:
            self._last_log = current
            forward, left, turn = velocity
            mode = '输入已使能' if self._enabled else '输入未使能'
            print(f'{mode} · 目标 前后 {forward:+.2f} m/s  横移 {left:+.2f} m/s  '
                  f'转向 {turn:+.2f} rad/s', flush=True)


def main(args=None):
    # 需要真实交互终端才能读取按键和切换输入模式；结束时必须恢复原termios设置。
    if not sys.stdin.isatty():
        raise SystemExit('请在交互式 Ubuntu 终端运行键盘节点。')
    saved = termios.tcgetattr(sys.stdin)
    terminal_output = sys.stdout.isatty()
    rclpy.init(args=args)
    node = None
    try:
        # cbreak使按键无需回车就可读取；节点的50ms定时器非阻塞检查终端输入。
        tty.setcbreak(sys.stdin.fileno())
        if terminal_output:
            # 兼容终端会报告焦点变化，并给粘贴文字加边界。
            print('\x1b[?1004h\x1b[?2004h', end='', flush=True)
        node = Keyboard()
        rclpy.spin(node)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        try:
            if node:
                if rclpy.ok():
                    node.stop_before_exit()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
            if terminal_output:
                print('\x1b[?2004l\x1b[?1004l', end='', flush=True)
