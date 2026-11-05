"""Ubuntu ROS 2 ↔ Mac 网关。全程不自动使能，也不自动恢复断线前的速度。"""
# 数据下行：/cmd_vel(TwistStamped) -> 本节点 -> RobotClient(TCP) -> Mac串口网关。
# 数据上行：STM32编码器/状态 -> Mac解析 -> 本节点 -> /joint_states、/robot/state。
# 这里只完成数据转换和通信；四轮目标分解、编码器测速、PI计算仍在STM32执行。
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
        # 这是ROS图中显示的节点名；ros2 run使用的可执行入口名是network_bridge。
        super().__init__('f401_network_bridge')
        # 可用--ros-args -p覆盖这些参数。host/port填写Mac网关地址，token_file
        # 读取Ubuntu本地保存的配对码；cpr是各输出轴每圈编码器计数，轮序FL/FR/RL/RR。
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('token_file', '~/.config/f401_robot/token')
        self.declare_parameter('cmd_timeout', 0.25)
        self.declare_parameter('cpr', [3960.0, 3960.0, 3960.0, 3960.0])
        self._state = CommandState(float(self.get_parameter('cmd_timeout').value))
        self._cpr = list(self.get_parameter('cpr').value)
        if len(self._cpr) != 4 or any(not math.isfinite(v) or v <= 0 for v in self._cpr):
            raise ValueError('cpr 需要四个正有限数，顺序 FL/FR/RL/RR')
        # _state保存有效运动目标；_client为网络连接；_arm_stamp_ns记录最近使能
        # 成功的ROS时间，防止使能前已经发布的旧指令在使能后突然生效。
        self._client = None
        self._arm_stamp_ns = None
        self._last_telemetry_key = None
        self._telemetry = None
        self._reason = 'disconnected'
        # String携带JSON状态便于诊断；JointState使用rad/rad/s描述四个轮关节。
        # 队列深度1只保留有限的待处理消息，接收端另做时间戳检查。
        self._status_pub = self.create_publisher(String, '/robot/state', 1)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 1)
        self._subscription = self.create_subscription(
            TwistStamped, '/cmd_vel', self._on_stamped, 1)
        # Trigger服务没有业务请求参数，回复success/message；连接、使能、停止
        # 分别是独立操作，建立TCP连接不直接授予运动许可。
        self.create_service(Trigger, '/robot/connect', self._connect_service)
        self.create_service(Trigger, '/robot/arm', self._arm_service)
        self.create_service(Trigger, '/robot/disarm', self._stop_service)
        # 同一单线程 executor 顺序使用客户端，避免两个请求同时消费 challenge。
        # 0.05秒定时器约20Hz发送最新目标；订阅回调仅接收/校验/缓存ROS消息。
        self.create_timer(0.05, self._tick)
        self._try_connect()

    def _drop_link(self, reason):
        # 先撤销本地运动意图，再关闭连接、清除旧遥测，后续状态不会沿用旧数据。
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
        # 每次显式连接都从停止状态开始。welcome中带有Mac已经校验过的会话遥测，
        # 处理成功后才把连接视为可用；异常时通过_drop_link统一清理。
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
        # ROS服务对象作为容器返回结果，实际TCP连接动作集中在_try_connect。
        del request
        response.success, response.message = self._try_connect()
        return response

    def _arm_service(self, request, response):
        # arm只开启接收新运动指令的许可，不恢复缓存速度。重复arm被解释为先停止，
        # 避免在正在运动时重建使能边界；真正使能成功以STM32返回状态为准。
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
        # 立即清本地目标，再请求网关向STM32发送STOP；success表示收到成功确认。
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
        # 订阅回调的msg就是ROS交给本节点的TwistStamped对象。
        # header.stamp拆成秒/纳秒，合成为纳秒后与本节点ROS时钟比较，计算消息年龄。
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if self._arm_stamp_ns is not None and stamp_ns <= self._arm_stamp_ns:
            age = float('inf')
        self._accept(msg.twist, age, msg.header.frame_id)

    def _accept(self, msg, age, frame):
        # 此处明确建立ROS字段与运动命令的关系：linear.x为前后m/s，linear.y为
        # 横移m/s，angular.z为转向rad/s；CommandState再转换成mm/s、mrad/s整数。
        # 使用单调时钟记录本机有效期限，避免系统日期调整改变本地超时计算。
        if not self._state.accept(msg.linear.x, msg.linear.y, msg.angular.z,
                                  time.monotonic(), stamp_age=age, frame=frame):
            self.get_logger().warning('速度消息无效、过期或超出遥控限速，目标归零')

    def _reply_reason(self, reply):
        # 网络事务OK只表示通信成功；若MCU故障位非零，优先显示真实底盘故障。
        mask = int((reply.get('telemetry') or {}).get('fault_mask', 0))
        if mask:
            return fault_description(mask)
        return str(reply.get('reason', 'unknown'))

    def _process_telemetry(self, telemetry):
        # Mac提供的age_ms是这份串口遥测在Mac缓存中的年龄，过期数据不再作为反馈。
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
        # 多次网络请求可能带回同一份MCU采样；只有会话/采样时间变化才发布关节反馈。
        if key != self._last_telemetry_key:
            # 暂用 ROS 接收时间打戳，MCU 原始时间保留在 /robot/state。
            # 尚未建立 MCU/ROS 时间同步，因此此阶段不发布 odom/TF。
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                        'rear_left_wheel_joint', 'rear_right_wheel_joint']
            msg.position, msg.velocity = position, velocity
            # position是轮轴累计角度，velocity是轮轴角速度；这里没有计算车身里程计。
            self._joint_pub.publish(msg)
            self._last_telemetry_key = key

    def _tick(self):
        # 将接收ROS话题与网络发送解耦：每周期取最新且未过期的缓存速度。
        # velocity()返回协议整数三元组，RobotClient把字段编码为JSON交给Mac；
        # 未使能时只查询状态。Mac负责再转为@V,...*CRC形式的串口帧。
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
                    # 指令被拒绝后撤销本地许可，发出明确STOP并更新停止后的反馈。
                    self._state.disarm()
                    stop = self._client.request('stop')
                    self._process_telemetry(stop.get('telemetry'))
            except (OSError, ValueError, KeyError, TypeError, RobotLinkError) as exc:
                self._drop_link(str(exc))
        msg = String()
        # 无论是否连通都发布状态，使上层能够区分断线、未使能及固件故障。
        msg.data = json.dumps({'connected': self._client is not None,
                               'armed': self._state.armed, 'reason': self._reason,
                               'telemetry': self._telemetry}, ensure_ascii=False)
        self._status_pub.publish(msg)

    def stop_before_exit(self):
        # 正常退出时先尝试停止底盘，再释放网络资源；清理不发送新运动目标。
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
    # rclpy.spin驱动订阅、服务和定时器回调；它持续运行，直到退出或发生异常。
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
