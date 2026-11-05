"""顺序请求/响应客户端；不自动重连，也不自动重新使能电机。"""
import argparse
from pathlib import Path
import socket
import time

from .protocol import JsonLineReader, ProtocolError, encode_json, velocity_fields


class RobotLinkError(RuntimeError):
    pass


class RobotClient:
    def __init__(self, host, port, token, timeout=0.2):
        self.host, self.port, self.token = host, port, token
        self.timeout = timeout
        self._sock = None
        self._reader = JsonLineReader()
        self._challenge = None

    def connect(self):
        if self._sock is not None:
            raise RobotLinkError("已经连接；重新连接前须先 close")
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._reader = JsonLineReader()
            self._sock.sendall(encode_json({"type": "auth", "token": self.token}))
            # 首次串口 H 握手和下一帧遥测最多需要两个串口周期。
            welcome = self._reader.receive(self._sock, max(self.timeout, 0.6), time.monotonic)
            if welcome.get("type") != "welcome" or welcome.get("version") != 1:
                raise ProtocolError(welcome.get("reason", "服务器未返回兼容 welcome"))
            self._set_challenge(welcome)
            return welcome
        except (OSError, ValueError, ProtocolError) as exc:
            self._abort()
            raise RobotLinkError("连接失败：" + str(exc)) from exc

    def _set_challenge(self, response):
        challenge = response.get("challenge")
        if not isinstance(challenge, str) or len(challenge) != 32:
            raise ProtocolError("服务器 challenge 无效")
        self._challenge = challenge

    def request(self, request_type, **fields):
        if self._sock is None or self._challenge is None:
            raise RobotLinkError("尚未连接")
        try:
            if request_type not in {"arm", "velocity", "stop", "status"}:
                raise ProtocolError("未知请求类型")
            if request_type == "velocity":
                velocity_fields(fields)
            if any(name in fields for name in ("type", "challenge")):
                raise ProtocolError("不允许覆盖协议字段")
            message = dict(fields, type=request_type, challenge=self._challenge)
            self._sock.sendall(encode_json(message))
            result = self._reader.receive(self._sock, self.timeout, time.monotonic)
            if result.get("type") != "reply" or type(result.get("ok")) is not bool:
                raise ProtocolError(result.get("reason", "服务器回复无效"))
            self._set_challenge(result)
            return result
        except (OSError, ValueError, ProtocolError) as exc:
            # 超时后不能重发：原请求可能已在 MCU 执行。关闭让服务器撤销驱动。
            self._abort()
            raise RobotLinkError("链路失效：" + str(exc)) from exc

    def _abort(self):
        sock, self._sock = self._sock, None
        self._challenge = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def close(self):
        if self._sock is not None and self._challenge is not None:
            try:
                self.request("stop")
            except RobotLinkError:
                pass
        self._abort()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def add_client_arguments(parser):
    parser.add_argument("--host", required=True, help="Mac 的 IP 地址")
    parser.add_argument("--port", type=int, default=8765, help="Mac 中转端口，默认 8765")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--token-file", type=Path, help="存有 Mac 配对码的文本文件")
    group.add_argument("--token", help="配对码；建议优先使用 --token-file，避免进入命令历史")


def token_from_arguments(args):
    try:
        value = args.token_file.read_text().strip() if args.token_file else args.token
    except OSError as exc:
        raise RobotLinkError("无法读取配对码文件：" + str(exc)) from exc
    if not value or len(value) > 256:
        raise RobotLinkError("配对码无效")
    return value
