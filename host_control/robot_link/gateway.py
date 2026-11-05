"""Mac 串口中转实现。此模块可用假串口测试，不在导入时打开硬件。"""
# 分为两层：SerialChannel 管 STM32 的串口会话，Gateway 管上位机的 TCP 会话。
# 网络命令到串口帧的联系写在 command()：velocity -> V，arm -> A，stop -> S。
# STM32 回传的 T 遥测先缓存在 SerialChannel，再随网络回复返回 Ubuntu/ROS 2 节点。
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
    # 表示串口读写、握手或确认时序失效；上层捕获后撤销本次网络控制会话。
    pass


class SerialChannel:
    def __init__(self, serial_port, command_timeout=0.17, telemetry_timeout=0.5):
        # serial_port 由入口创建，也可以是测试提供的假串口；本类不选择系统设备路径。
        # command_timeout 是单次指令确认期限，telemetry_timeout 是遥测最大年龄，单位均为秒。
        self.port = serial_port
        self.command_timeout = command_timeout
        self.telemetry_timeout = telemetry_timeout
        self.session = 0
        self.seq = 0
        # Condition 保护收发线程共用的 ACK/遥测，并让等待者在新回包到达时被唤醒。
        # command_lock 保证一次只有一条指令事务，避免两个调用方同时改写正在等待的序号。
        self._condition = threading.Condition()
        self._command_lock = threading.Lock()
        self._stop = threading.Event()
        self._fatal = None
        # _telemetry 是最新一帧有效反馈，_telemetry_at 是 Mac 收到它的单调时钟时刻。
        # _awaiting=(session, seq) 标识本次命令，_ack 保存与它精确匹配的确认。
        self._telemetry = None
        self._telemetry_at = 0.0
        self._awaiting = None
        self._ack = None
        self._thread = threading.Thread(target=self._read_loop, name="serial-reader", daemon=True)

    def start(self):
        # 串口接收独立进行，所以前台可以等待 ACK，而不会停止读取反馈。
        self._thread.start()

    def _read_loop(self):
        # 串口 read() 不保证一次返回一整帧，必须逐字节缓存，遇到换行才开始解析。
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
                                # 忽略其他会话或旧序号的 ACK，不能用迟到回包确认新命令。
                                if self._awaiting == (event["session"], event["seq"]):
                                    self._ack = event
                            elif event["session"] == self.session:
                                # 遥测只接纳当前串口会话；旧会话数据不会覆盖当前运行状态。
                                self._telemetry = event
                                self._telemetry_at = time.monotonic()
                            self._condition.notify_all()
                    else:
                        pending.append(byte)
                        if len(pending) > 511:
                            raise SerialLinkError("串口收到过长行")
        except Exception as exc:
            if not self._stop.is_set():
                # 接收线程不能直接把异常抛到前台，记录故障并唤醒等待者，由前台统一处理。
                with self._condition:
                    self._fatal = str(exc)
                    self._condition.notify_all()

    def _check_health(self):
        # 在发命令、等回包和取遥测前检查接收线程是否已经失效，避免一直等待无结果。
        if self._fatal is not None:
            raise SerialLinkError(self._fatal)
        if self._stop.is_set():
            raise SerialLinkError("串口中转已停止")

    def _write(self, payload):
        # 文字内容先由 protocol 加帧头/CRC/换行；127字节上限与固件的接收边界对应。
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
        # H 是建立新串口会话的握手；随机非零会话号隔离上次连接留下的指令与反馈。
        # 网关启动、每个网络客户端通过认证后都会握手，固件由此从停止/未使能状态开始。
        with self._command_lock:
            with self._condition:
                self._check_health()
                self.session = secrets.randbelow(0xFFFFFFFF) + 1
                self.seq = 0
                self._telemetry = None
                self._telemetry_at = 0.0
            # 清掉旧快照后必须收到本次 H 的 ACK 和新遥测，才把握手判定为成功。
            result = self._transaction("H", [], timeout=0.35)
            if not result["ok"]:
                raise SerialLinkError("固件 H 握手被拒绝：" + result["reason"])
            return result["telemetry"]

    def _transaction(self, code, values, timeout=None):
        """A/H/S 等待本次消费后的遥测；V 用新 ACK 配合有效期内的会话遥测。"""
        # 先登记等待目标，再写串口，防止设备很快回复时接收线程还不知道要等哪条命令。
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
                    # 否定 ACK 也是有效回复，返回拒绝原因；没有确认时才继续等待或超时。
                    if self._ack["result"] != "OK":
                        return {"ok": False, "reason": self._ack["result"], "telemetry": self._snapshot_unchecked()}
                    current = self._telemetry
                    # H/A/S 改变会话或使能状态，要求快照明确反映本条 seq 且在发送后收到。
                    # V 的连续控制频率较高，匹配本条 ACK 并持有未过期遥测即可返回。
                    fresh = current is not None and current["session"] == self.session and \
                        time.monotonic() - self._telemetry_at < self.telemetry_timeout and \
                        (code == "V" or (current["last_seq"] == self.seq and self._telemetry_at >= sent_at))
                    if fresh:
                        telemetry = self._snapshot_unchecked()
                        if code in ("A", "V") and (not telemetry["armed"] or telemetry["fault_mask"] != 0):
                            # ACK=OK 仍需结合底盘状态：故障或未使能时不能向上层报告运动获准。
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
                # wait() 等待期间释放 Condition 的锁，接收线程才能更新 ACK/遥测并唤醒它。

    def _snapshot_unchecked(self):
        # 调用方持有 Condition 锁；深拷贝防止网络层修改缓存中的四轮列表。
        # age_ms 是 Mac 侧接收年龄，不是将 MCU 的 mcu_ms 与 ROS 时间相减得到的值。
        if self._telemetry is None:
            return None
        value = copy.deepcopy(self._telemetry)
        value.pop("type", None)
        value["age_ms"] = max(0, int((time.monotonic() - self._telemetry_at) * 1000))
        return value

    def telemetry(self):
        # status 查询使用这里的最新缓存；没有反馈、反馈过期或会话不符都视为链路失效。
        with self._condition:
            self._check_health()
            result = self._snapshot_unchecked()
            if result is None or result["session"] != self.session or result["age_ms"] >= self.telemetry_timeout * 1000:
                raise SerialLinkError("串口遥测已过期或 session 不符")
            return result

    def command(self, kind, **fields):
        # 这是“网络消息如何变成串口”的关键映射：可读的 type 转为单字母命令码。
        code = {"arm": "A", "velocity": "V", "stop": "S"}.get(kind)
        if code is None:
            raise ProtocolError("未知串口控制类型")
        values = velocity_fields(fields) if code == "V" else []
        # velocity_fields 按 vx_mm_s、vy_mm_s、wz_mrad_s 的顺序取三个整数。
        # 最终 _transaction 组成 V,session,seq,vx,vy,wz，再交给 _write 加 CRC 帧。
        with self._command_lock:
            self.telemetry()
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            # 序号按32位回绕，与固件保持一致；成功发起新命令前必须有有效的当前遥测。
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
        # 先尝试停止，再通知后台线程退出；关闭串口使未完成的 read 尽快结束。
        self.stop_best_effort()
        self._stop.set()
        try:
            self.port.close()
        finally:
            if self._thread.is_alive():
                self._thread.join(timeout=0.3)


def load_or_create_token(path):
    # 两端使用同一配对码完成 TCP 认证；此函数仅在网关入口显式调用时访问本地文件。
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        # O_EXCL 保证只在文件不存在时创建，0600 使新文件仅当前用户可读写。
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
    # 32随机字节编码成64个十六进制字符，持久化后可供后续启动复用。
    with os.fdopen(fd, "w") as stream:
        stream.write(token + "\n")
    return token


class Gateway:
    def __init__(self, channel, token, challenge_ttl=0.3):
        # token 用于首次配对，challenge 是认证后每条请求的一次性凭据。
        # challenge_ttl 单位秒，限制客户端持有下一次请求许可的时间。
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
        # 正常断开和协议出错都用同一收尾路径，优先等待明确的停止确认。
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
            # 第一个网络包必须是 auth；认证完成前不为此客户端重建 STM32 控制会话。
            auth = reader.receive(conn, self.challenge_ttl, time.monotonic)
            if auth.get("type") != "auth" or not isinstance(auth.get("token"), str) or \
                    not hmac.compare_digest(auth["token"].encode("utf-8"), self.token.encode("utf-8")):
                raise ProtocolError("认证失败")
            telemetry = self.channel.begin_session()
            established = True
            # welcome 返回协议版本、初始遥测，以及首次业务请求要带回的一次性 challenge。
            challenge = secrets.token_hex(16)
            issued = time.monotonic()
            conn.sendall(encode_json({"type": "welcome", "version": 1, "challenge": challenge, "telemetry": telemetry}))
            while True:
                # 以签发时刻计算剩余期限，TCP 分片和慢速发送不能延长 challenge 的有效期。
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
                    # 状态查询只取串口接收线程缓存，不另发一条固件查询命令。
                    response = {"ok": True, "reason": "OK", "telemetry": self.channel.telemetry()}
                elif kind in ("arm", "stop"):
                    response = self.channel.command(kind)
                elif kind == "velocity":
                    # 这里从 JSON 对象明确取出三个速度字段，交给串口层编码和确认。
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
                # 每条回复都包含控制结果、遥测和下一枚 challenge，形成顺序请求/响应链。
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
            # 只有已经建立串口控制会话的连接才执行控制清理；随后始终释放 TCP 连接。
            if established:
                self._cleanup_control()
            conn.close()

    def serve(self, bind, port):
        # TCP 监听端口对 Ubuntu 提供入口；一次串行服务一个客户端，不并行争用串口。
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

