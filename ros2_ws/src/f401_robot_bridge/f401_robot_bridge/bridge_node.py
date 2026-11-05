"""Ubuntu ROS 2 ↔ Mac 网关。全程不自动使能，也不自动恢复断线前的速度。"""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_link.client import RobotClient, RobotLinkError
from .control_state import CommandState, fault_description, joint_values


class NetworkBridge(Node):
    def __init__(self):
        super().__init__('f401_network_bridge')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('token_file', '~/.config/f401_robot/token')
        self.declare_parameter('cmd_timeout', 0.25)
        self.declare_parameter('cpr', [3960.0, 3960.0, 3960.0, 3960.0])
        self._state = CommandState(float(self.get_parameter('cmd_timeout').value))
        self._cpr = list(self.get_parameter('cpr').value)
        if len(self._cpr) != 4 or any(not math.isfinite(v) or v <= 0 for v in self._cpr):
            raise ValueError('cpr 需要四个正有限数，顺序 FL/FR/RL/RR')
        self._client = None
        self._arm_stamp_ns = None
        self._last_telemetry_key = None
        self._telemetry = None
        self._reason = 'disconnected'
        self._status_pub = self.create_publisher(String, '/robot/state', 1)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 1)
        self._subscription = self.create_subscription(
            TwistStamped, '/cmd_vel', self._on_stamped, 1)
        self.create_service(Trigger, '/robot/connect', self._connect_service)
        self.create_service(Trigger, '/robot/arm', self._arm_service)
        self.create_service(Trigger, '/robot/disarm', self._stop_service)
        # 同一单线程 executor 顺序使用客户端，避免两个请求同时消费 challenge。
        self.create_timer(0.05, self._tick)
        self._try_connect()

    def _drop_link(self, reason):
        self._state.disarm()
        self._arm_stamp_ns = None
        client, self._client = self._client, None
        self._telemetry = None
        self._last_telemetry_key = None
        self._reason = str(reason)
        if client:
            client.close()
        self.get_logger().warning('链路停止：' + self._reason + '；恢复后需显式 connect/arm')

    def _try_connect(self):
        self._drop_link('connecting')
        client = None
        try:
            token = Path(str(self.get_parameter('token_file').value)).expanduser().read_text().strip()
            if not token:
                raise ValueError('配对码文件为空')
            client = RobotClient(str(self.get_parameter('host').value),
                                 int(self.get_parameter('port').value), token, timeout=0.2)
            welcome = client.connect()
            self._client = client
            self._process_telemetry(welcome.get('telemetry'))
            self._reason = 'connected_disarmed'
            self.get_logger().info('Mac 网关已连接，当前未使能')
            return True, self._reason
        except (OSError, ValueError, KeyError, TypeError, RobotLinkError) as exc:
            if client:
                client.close()
            self._drop_link(str(exc))
            return False, self._reason

    def _connect_service(self, request, response):
        del request
        response.success, response.message = self._try_connect()
        return response

    def _arm_service(self, request, response):
        del request
        was_armed = self._state.armed
        self._state.disarm()
        self._arm_stamp_ns = None
        if not self._client:
            response.success, response.message = False, '未连接，先调用 /robot/connect'
            return response
        try:
            if was_armed:
                reply = self._client.request('stop')
                self._process_telemetry(reply.get('telemetry'))
                response.success, response.message = False, '已停止，请等车轮停稳后再次使能'
                return response
            reply = self._client.request('arm')
            self._process_telemetry(reply.get('telemetry'))
            response.success = bool(reply.get('ok') and self._telemetry.get('armed') and
                                    not self._telemetry.get('fault_mask'))
            response.message = self._reply_reason(reply)
            if response.success:
                self._state.arm()
                # ARM 等待串口确认期间 DDS 可能积压一条旧消息；只接受此时之后发布的指令。
                self._arm_stamp_ns = self.get_clock().now().nanoseconds
            else:
                stop = self._client.request('stop')
                self._process_telemetry(stop.get('telemetry'))
                response.message += '；已解除使能'
        except (OSError, ValueError, KeyError, TypeError, RobotLinkError) as exc:
            self._drop_link(str(exc))
            response.success, response.message = False, self._reason
        return response

    def _stop_service(self, request, response):
        del request
        self._state.disarm()
        self._arm_stamp_ns = None
        response.success, response.message = False, '未连接；STM32 仍有本地超时停止'
        if self._client:
            try:
                reply = self._client.request('stop')
                self._process_telemetry(reply.get('telemetry'))
                response.success = bool(reply.get('ok'))
                response.message = self._reply_reason(reply)
            except (OSError, ValueError, KeyError, TypeError, RobotLinkError) as exc:
                self._drop_link(str(exc))
                response.message = self._reason
        return response

    def _on_stamped(self, msg):
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if self._arm_stamp_ns is not None and stamp_ns <= self._arm_stamp_ns:
            age = float('inf')
        self._accept(msg.twist, age, msg.header.frame_id)

    def _accept(self, msg, age, frame):
        if not self._state.accept(msg.linear.x, msg.linear.y, msg.angular.z,
                                  time.monotonic(), stamp_age=age, frame=frame):
            self.get_logger().warning('速度消息无效、过期或超出遥控限速，目标归零')

    def _reply_reason(self, reply):
        mask = int((reply.get('telemetry') or {}).get('fault_mask', 0))
        if mask:
            return fault_description(mask)
        return str(reply.get('reason', 'unknown'))

    def _process_telemetry(self, telemetry):
        if not isinstance(telemetry, dict):
            raise ValueError('Mac 未提供有效遥测')
        age = float(telemetry.get('age_ms', math.inf))
        if not math.isfinite(age) or not 0 <= age < 500:
            raise ValueError('STM32 遥测已过期')
        position, velocity = joint_values(telemetry, self._cpr)
        old_fault = int((self._telemetry or {}).get('fault_mask', 0))
        self._telemetry = telemetry
        mask = int(telemetry.get('fault_mask', 0))
        if mask and mask != old_fault:
            self.get_logger().warning('底盘已停止：' + fault_description(mask) + '；排查后复位，不会自动清故障')
        if telemetry.get('fault_mask') or not telemetry.get('armed'):
            self._state.disarm()
            self._arm_stamp_ns = None
        key = (telemetry['session'], telemetry['mcu_ms'])
        if key != self._last_telemetry_key:
            # 暂用 ROS 接收时间打戳，MCU 原始时间保留在 /robot/state。
            # 尚未建立 MCU/ROS 时间同步，因此此阶段不发布 odom/TF。
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                        'rear_left_wheel_joint', 'rear_right_wheel_joint']
            msg.position, msg.velocity = position, velocity
            self._joint_pub.publish(msg)
            self._last_telemetry_key = key

    def _tick(self):
        if self._client:
            try:
                if self._state.armed:
                    vx, vy, wz = self._state.velocity(time.monotonic())
                    reply = self._client.request('velocity', vx_mm_s=vx, vy_mm_s=vy, wz_mrad_s=wz)
                else:
                    reply = self._client.request('status')
                self._process_telemetry(reply.get('telemetry'))
                self._reason = self._reply_reason(reply)
                if not reply.get('ok'):
                    self._state.disarm()
                    stop = self._client.request('stop')
                    self._process_telemetry(stop.get('telemetry'))
            except (OSError, ValueError, KeyError, TypeError, RobotLinkError) as exc:
                self._drop_link(str(exc))
        msg = String()
        msg.data = json.dumps({'connected': self._client is not None,
                               'armed': self._state.armed, 'reason': self._reason,
                               'telemetry': self._telemetry}, ensure_ascii=False)
        self._status_pub.publish(msg)

    def stop_before_exit(self):
        self._state.disarm()
        self._arm_stamp_ns = None
        if self._client:
            try:
                self._client.request('stop')
            except (OSError, ValueError, RobotLinkError):
                pass
            self._client.close()
            self._client = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NetworkBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.stop_before_exit()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
