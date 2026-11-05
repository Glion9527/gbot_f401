"""Mac 串口中转实现。此模块可用假串口测试，不在导入时打开硬件。"""
import copy
import hmac
import os
from pathlib import Path
import secrets
import socket
import threading
import time

from .protocol import JsonLineReader, ProtocolError, encode_json, parse_serial, serial_frame, velocity_fields


class SerialLinkError(RuntimeError):
    pass


class SerialChannel:
    def __init__(self, serial_port, command_timeout=0.17, telemetry_timeout=0.5):
        self.port = serial_port
        self.command_timeout = command_timeout
        self.telemetry_timeout = telemetry_timeout
        self.session = 0
        self.seq = 0
        self._condition = threading.Condition()
        self._command_lock = threading.Lock()
        self._stop = threading.Event()
        self._fatal = None
        self._telemetry = None
        self._telemetry_at = 0.0
        self._awaiting = None
        self._ack = None
        self._thread = threading.Thread(target=self._read_loop, name="serial-reader", daemon=True)

    def start(self):
        self._thread.start()

    def _read_loop(self):
        pending = bytearray()
        try:
            while not self._stop.is_set():
                data = self.port.read(128)
                if not data:
                    if getattr(self.port, "is_open", True) is False and not self._stop.is_set():
                        raise SerialLinkError("串口已关闭")
                    continue
                for byte in data:
                    if byte == 10:
                        raw, pending = bytes(pending), bytearray()
                        # CubeMX/旧程序的普通文本不算握手成功，但可以忽略以便诊断。
                        if not raw.startswith(b"@"):
                            continue
                        event = parse_serial(raw)
                        with self._condition:
                            if event["type"] == "ack":
                                if self._awaiting == (event["session"], event["seq"]):
                                    self._ack = event
                            elif event["session"] == self.session:
                                self._telemetry = event
                                self._telemetry_at = time.monotonic()
                            self._condition.notify_all()
                    else:
                        pending.append(byte)
                        if len(pending) > 511:
                            raise SerialLinkError("串口收到过长行")
        except Exception as exc:
            if not self._stop.is_set():
                with self._condition:
                    self._fatal = str(exc)
                    self._condition.notify_all()

    def _check_health(self):
        if self._fatal is not None:
            raise SerialLinkError(self._fatal)
        if self._stop.is_set():
            raise SerialLinkError("串口中转已停止")

    def _write(self, payload):
        data = serial_frame(payload)
        if len(data.rstrip(b"\n")) > 127:
            raise SerialLinkError("串口命令过长")
        try:
            written = self.port.write(data)
            if written != len(data):
                raise SerialLinkError("串口未完整写入命令")
        except Exception as exc:
            raise SerialLinkError("串口写入失败：" + str(exc)) from exc

    def begin_session(self):
        with self._command_lock:
            with self._condition:
                self._check_health()
                self.session = secrets.randbelow(0xFFFFFFFF) + 1
                self.seq = 0
                self._telemetry = None
                self._telemetry_at = 0.0
            result = self._transaction("H", [], timeout=0.35)
            if not result["ok"]:
                raise SerialLinkError("固件 H 握手被拒绝：" + result["reason"])
            return result["telemetry"]

    def _transaction(self, code, values, timeout=None):
        """A/H/S 等待本次消费后的遥测；V 用新 ACK 配合有效期内的会话遥测。"""
        timeout = self.command_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        with self._condition:
            self._check_health()
            self._awaiting = (self.session, self.seq)
            self._ack = None
            sent_at = time.monotonic()
        self._write(",".join(str(value) for value in [code, self.session, self.seq] + values))
        with self._condition:
            while True:
                self._check_health()
                if self._ack is not None:
                    if self._ack["result"] != "OK":
                        return {"ok": False, "reason": self._ack["result"], "telemetry": self._snapshot_unchecked()}
                    current = self._telemetry
                    fresh = current is not None and current["session"] == self.session and \
                        time.monotonic() - self._telemetry_at < self.telemetry_timeout and \
                        (code == "V" or (current["last_seq"] == self.seq and self._telemetry_at >= sent_at))
                    if fresh:
                        telemetry = self._snapshot_unchecked()
                        if code in ("A", "V") and (not telemetry["armed"] or telemetry["fault_mask"] != 0):
                            return {"ok": False, "reason": "DISARMED", "telemetry": telemetry}
                        return {"ok": True, "reason": "OK", "telemetry": telemetry}
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    latest = self._snapshot_unchecked()
                    detail = "none" if latest is None else "seq=%s armed=%s fault=%s age_ms=%s" % (
                        latest["last_seq"], latest["armed"], latest["fault_mask"], latest["age_ms"])
                    raise SerialLinkError("等待本次 ACK/新遥测超时 [cmd=%s seq=%d ack=%s telemetry=%s]" % (
                        code, self.seq, self._ack["result"] if self._ack else "none", detail))
                self._condition.wait(remaining)

    def _snapshot_unchecked(self):
        if self._telemetry is None:
            return None
        value = copy.deepcopy(self._telemetry)
        value.pop("type", None)
        value["age_ms"] = max(0, int((time.monotonic() - self._telemetry_at) * 1000))
        return value

    def telemetry(self):
        with self._condition:
            self._check_health()
            result = self._snapshot_unchecked()
            if result is None or result["session"] != self.session or result["age_ms"] >= self.telemetry_timeout * 1000:
                raise SerialLinkError("串口遥测已过期或 session 不符")
            return result

    def command(self, kind, **fields):
        code = {"arm": "A", "velocity": "V", "stop": "S"}.get(kind)
        if code is None:
            raise ProtocolError("未知串口控制类型")
        values = velocity_fields(fields) if code == "V" else []
        with self._command_lock:
            self.telemetry()
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            return self._transaction(code, values)

    def stop_best_effort(self):
        # 退出时不等 ACK，减少通信已经故障时的等待；MCU 本地 300ms 超时仍有效。
        with self._command_lock:
            if self.session == 0:
                return
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            try:
                self._write("S,%d,%d" % (self.session, self.seq))
            except SerialLinkError:
                pass

    def close(self):
        self.stop_best_effort()
        self._stop.set()
        try:
            self.port.close()
        finally:
            if self._thread.is_alive():
                self._thread.join(timeout=0.3)


def load_or_create_token(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        if path.is_symlink() or not path.is_file():
            raise ValueError("配对码路径必须是普通文件")
        token = path.read_text().strip()
        if len(token) < 32 or len(token) > 256:
            raise ValueError("已有配对码无效；请检查文件")
        os.chmod(path, 0o600)
        return token
    token = secrets.token_hex(32)
    with os.fdopen(fd, "w") as stream:
        stream.write(token + "\n")
    return token


class Gateway:
    def __init__(self, channel, token, challenge_ttl=0.3):
        self.channel, self.token = channel, token
        self.challenge_ttl = challenge_ttl

    def _confirmed_stop(self):
        # STM32的STOP会清接收队列。必须等当前S的ACK和新T后，才允许下一帧；
        # 连发两个未确认S可能让前一个S把后一个S/H清掉，造成假性通信超时。
        response = self.channel.command("stop")
        if not response["ok"]:
            raise SerialLinkError("STM32拒绝停止命令：" + response["reason"])
        return response

    def _cleanup_control(self):
        try:
            self._confirmed_stop()
        except (OSError, ValueError, SerialLinkError):
            # 链路确已故障时仅尽力停一次，不自动重发运动请求。
            self.channel.stop_best_effort()

    def serve_client(self, conn):
        """一个连接持有一个会话；无自动 ARM、无自动重发或断线恢复。"""
        reader = JsonLineReader()
        established = False
        try:
            auth = reader.receive(conn, self.challenge_ttl, time.monotonic)
            if auth.get("type") != "auth" or not isinstance(auth.get("token"), str) or \
                    not hmac.compare_digest(auth["token"].encode("utf-8"), self.token.encode("utf-8")):
                raise ProtocolError("认证失败")
            telemetry = self.channel.begin_session()
            established = True
            challenge = secrets.token_hex(16)
            issued = time.monotonic()
            conn.sendall(encode_json({"type": "welcome", "version": 1, "challenge": challenge, "telemetry": telemetry}))
            while True:
                remaining = issued + self.challenge_ttl - time.monotonic()
                if remaining <= 0:
                    raise ProtocolError("challenge 已过期")
                message = reader.receive(conn, remaining, time.monotonic)
                if time.monotonic() - issued >= self.challenge_ttl or \
                        not isinstance(message.get("challenge"), str) or \
                        not hmac.compare_digest(message["challenge"].encode("utf-8"), challenge.encode("utf-8")):
                    raise ProtocolError("challenge 已过期或不匹配")
                # 消耗旧 challenge，处理完才发新的，提前排队的旧请求无法执行。
                challenge = ""
                kind = message.get("type")
                if kind == "status":
                    response = {"ok": True, "reason": "OK", "telemetry": self.channel.telemetry()}
                elif kind in ("arm", "stop"):
                    response = self.channel.command(kind)
                elif kind == "velocity":
                    velocity_fields(message)
                    response = self.channel.command(kind, vx_mm_s=message["vx_mm_s"],
                                                    vy_mm_s=message["vy_mm_s"], wz_mrad_s=message["wz_mrad_s"])
                else:
                    raise ProtocolError("未知网络请求")
                if not response["ok"]:
                    stopped = self._confirmed_stop()
                    # 保留原拒绝原因，但反馈必须是已确认停止后的状态。
                    response["telemetry"] = stopped["telemetry"]
                challenge = secrets.token_hex(16)
                issued = time.monotonic()
                conn.sendall(encode_json(dict(response, type="reply", challenge=challenge)))
        except (OSError, ValueError, ProtocolError, SerialLinkError) as exc:
            if established:
                established = False
                self._cleanup_control()
            print("控制会话停止：%s" % exc, flush=True)
            try:
                conn.sendall(encode_json({"type": "error", "reason": str(exc)}))
            except OSError:
                pass
        finally:
            if established:
                self._cleanup_control()
            conn.close()

    def serve(self, bind, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((bind, port))
            listener.listen(1)
            print("中转已监听 %s:%d；连接本身不会使能电机。" % (bind, port), flush=True)
            while True:
                conn, address = listener.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("客户端连接：%s；等待认证。" % address[0], flush=True)
                self.serve_client(conn)
                print("客户端已退出；已撤销运动请求。", flush=True)

