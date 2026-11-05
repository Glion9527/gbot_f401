"""顺序请求/响应客户端；不自动重连，也不自动重新使能电机。"""
# ROS 2 桥接、命令行键盘与 probe 都使用这个接口访问 Mac，无需自己操作串口。
# 每次 request() 先发送一行 JSON，再等待一行回复；调用方应顺序调用，不能并发请求。
import argparse
from pathlib import Path
import socket
import time

from .protocol import JsonLineReader, ProtocolError, encode_json, velocity_fields


class RobotLinkError(RuntimeError):
    # 将底层 socket、协议等错误统一为上层脚本可以捕获的链路异常。
    pass


class RobotClient:
    def __init__(self, host, port, token, timeout=0.2):
        # host/port 是 Mac 网关监听地址；token 是两端约定的配对码。
        # timeout 单位为秒；_challenge 保存网关上次回复给出的下一次请求凭据。
        self.host, self.port, self.token = host, port, token
        self.timeout = timeout
        self._sock = None
        self._reader = JsonLineReader()
        self._challenge = None

    def connect(self):
        # TCP 建连只说明网络接通，还要完成配对认证并验证 welcome 的协议版本。
        if self._sock is not None:
            raise RobotLinkError("已经连接；重新连接前须先 close")
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            # 小控制包不等待 TCP 合并后再发送，减少交互延迟；消息边界仍由换行确定。
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._reader = JsonLineReader()
            self._sock.sendall(encode_json({"type": "auth", "token": self.token}))
            # 首次串口 H 握手和下一帧遥测最多需要两个串口周期。
            welcome = self._reader.receive(self._sock, max(self.timeout, 0.6), time.monotonic)
            if welcome.get("type") != "welcome" or welcome.get("version") != 1:
                raise ProtocolError(welcome.get("reason", "服务器未返回兼容 welcome"))
            self._set_challenge(welcome)
            # welcome 同时携带 STM32 遥测；上层可据此显示当前会话和未使能状态。
            return welcome
        except (OSError, ValueError, ProtocolError) as exc:
            self._abort()
            raise RobotLinkError("连接失败：" + str(exc)) from exc

    def _set_challenge(self, response):
        # 每份回复提供新的 challenge，下一次请求必须带回它；旧请求不能重复使用。
        challenge = response.get("challenge")
        if not isinstance(challenge, str) or len(challenge) != 32:
            raise ProtocolError("服务器 challenge 无效")
        self._challenge = challenge

    def request(self, request_type, **fields):
        # 上层使用可读的命令名（如 velocity），Mac 再映射成串口的 V/A/S 命令。
        # velocity 字段单位已经是 mm/s、mrad/s，这一层仅校验，不做 m/s 的换算。
        if self._sock is None or self._challenge is None:
            raise RobotLinkError("尚未连接")
        try:
            if request_type not in {"arm", "velocity", "stop", "status"}:
                raise ProtocolError("未知请求类型")
            if request_type == "velocity":
                velocity_fields(fields)
            if any(name in fields for name in ("type", "challenge")):
                raise ProtocolError("不允许覆盖协议字段")
            # 例如速度字段组成 {type: velocity, vx_mm_s: 80, ...}；encode_json 加换行。
            message = dict(fields, type=request_type, challenge=self._challenge)
            self._sock.sendall(encode_json(message))
            result = self._reader.receive(self._sock, self.timeout, time.monotonic)
            if result.get("type") != "reply" or type(result.get("ok")) is not bool:
                raise ProtocolError(result.get("reason", "服务器回复无效"))
            # reply 格式有效不等于动作获准；ok=False 也会正常返回，由调用方决定停机等处理。
            self._set_challenge(result)
            return result
        except (OSError, ValueError, ProtocolError) as exc:
            # 超时后不能重发：原请求可能已在 MCU 执行。关闭让服务器撤销驱动。
            self._abort()
            raise RobotLinkError("链路失效：" + str(exc)) from exc

    def _abort(self):
        # 清空当前连接与请求凭据；shutdown 尽量唤醒对端，close 释放本机 socket。
        sock, self._sock = self._sock, None
        self._challenge = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def close(self):
        # 正常退出先尝试通知 STM32 停止；链路已损坏时仍必须释放本地连接。
        if self._sock is not None and self._challenge is not None:
            try:
                self.request("stop")
            except RobotLinkError:
                pass
        self._abort()

    def __enter__(self):
        # 支持 with RobotClient(...) as client，在代码块结束时自动调用 __exit__ 清理。
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def add_client_arguments(parser):
    # 三个命令行入口共用参数定义，保证连接同一网关时使用一致的 host/port/token 含义。
    parser.add_argument("--host", required=True, help="Mac 的 IP 地址")
    parser.add_argument("--port", type=int, default=8765, help="Mac 中转端口，默认 8765")
    group = parser.add_mutually_exclusive_group(required=True)
    # 配对码文件和直接传入字符串二选一；互斥组同时要求至少选择一种。
    group.add_argument("--token-file", type=Path, help="存有 Mac 配对码的文本文件")
    group.add_argument("--token", help="配对码；建议优先使用 --token-file，避免进入命令历史")


def token_from_arguments(args):
    # 这里只读用户明确指定的文件；去掉末尾换行，得到认证请求需要的原始配对码。
    try:
        value = args.token_file.read_text().strip() if args.token_file else args.token
    except OSError as exc:
        raise RobotLinkError("无法读取配对码文件：" + str(exc)) from exc
    if not value or len(value) > 256:
        raise RobotLinkError("配对码无效")
    return value
