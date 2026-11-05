"""串口 CRC 帧和有限长度 JSON 行；所有速度字段使用整数单位。"""
import json
import socket


class ProtocolError(ValueError):
    pass


def crc16(data):
    """CRC16-CCITT-FALSE；标准测试向量 123456789 -> 0x29B1。"""
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def serial_frame(payload):
    data = payload.encode("ascii")
    if any(char in data for char in (b"\n", b"\r", b"*", b"@")):
        raise ProtocolError("串口 payload 含有非法字符")
    return b"@" + data + b"*%04X\n" % crc16(data)


def serial_payload(line):
    raw = line.rstrip(b"\r\n")
    if len(raw) > 511 or len(raw) < 7 or raw[:1] != b"@" or raw[-5:-4] != b"*":
        raise ProtocolError("串口帧格式或长度不符")
    data, digits = raw[1:-5], raw[-4:]
    if any(x not in b"0123456789ABCDEF" for x in digits) or int(digits, 16) != crc16(data):
        raise ProtocolError("串口 CRC 校验失败")
    try:
        return data.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolError("串口 payload 不是 ASCII") from exc


def decimal(value, lo, hi):
    if not isinstance(value, str) or not value or value in ("+", "-"):
        raise ProtocolError("无效整数")
    digits = value[1:] if value[:1] == "-" else value
    if not digits.isascii() or not digits.isdigit():
        raise ProtocolError("整数必须使用十进制数字")
    number = int(value)
    if number < lo or number > hi:
        raise ProtocolError("整数越界")
    return number


def uint32(value):
    return decimal(value, 0, 0xFFFFFFFF)


def parse_serial(line):
    parts = serial_payload(line).split(",")
    if parts[0] == "ACK" and len(parts) == 4:
        if parts[3] not in {"OK", "DISARMED", "BUSY", "FAULT", "REJECTED", "INVALID"}:
            raise ProtocolError("ACK 结果未知")
        return {"type": "ack", "session": uint32(parts[1]), "seq": uint32(parts[2]), "result": parts[3]}
    if parts[0] == "T" and len(parts) == 22:
        armed = decimal(parts[3], 0, 1)
        wheels = []
        for index, name in enumerate(("FL", "FR", "RL", "RR")):
            wheels.append({
                "id": name,
                "count": decimal(parts[6 + index], -(1 << 63), (1 << 63) - 1),
                "rpm_x10": decimal(parts[10 + index], -(1 << 31), (1 << 31) - 1),
                "target_x10": decimal(parts[14 + index], -(1 << 31), (1 << 31) - 1),
                "pwm": decimal(parts[18 + index], -1000, 1000),
            })
        return {"type": "telemetry", "session": uint32(parts[1]), "mcu_ms": uint32(parts[2]),
                "armed": bool(armed), "last_seq": uint32(parts[4]),
                "fault_mask": decimal(parts[5], 0, 255), "wheels": wheels}
    raise ProtocolError("未知串口消息或字段数不符")


def encode_json(message):
    try:
        data = json.dumps(message, separators=(",", ":"), ensure_ascii=True, allow_nan=False).encode("ascii")
    except (ValueError, TypeError) as exc:
        raise ProtocolError("无效 JSON 消息") from exc
    if len(data) > 4096:
        raise ProtocolError("JSON 消息超过 4096 字节")
    return data + b"\n"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("JSON 包含重复键")
        result[key] = value
    return result


def decode_json(data):
    if len(data) > 4096:
        raise ProtocolError("JSON 消息超过 4096 字节")
    try:
        obj = json.loads(data.decode("utf-8"), object_pairs_hook=_unique_object,
                         parse_constant=lambda value: (_ for _ in ()).throw(ProtocolError("不允许 NaN/Inf")))
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ProtocolError("无法解析 JSON 消息") from exc
    if not isinstance(obj, dict):
        raise ProtocolError("JSON 消息必须是对象")
    return obj


class JsonLineReader:
    """保留 TCP 分片，但限制整条消息的字节数；半帧不因多次 recv 延长超时。"""
    def __init__(self):
        self.buffer = bytearray()

    def receive(self, sock, timeout, clock):
        deadline = clock() + timeout
        while True:
            end = self.buffer.find(b"\n")
            if end >= 0:
                if end > 4096:
                    raise ProtocolError("JSON 行过长")
                line = bytes(self.buffer[:end])
                del self.buffer[:end + 1]
                return decode_json(line)
            if len(self.buffer) > 4096:
                raise ProtocolError("JSON 行过长")
            remaining = deadline - clock()
            if remaining <= 0:
                raise TimeoutError("等待网络消息超时")
            sock.settimeout(remaining)
            try:
                data = sock.recv(1024)
            except socket.timeout as exc:
                raise TimeoutError("等待网络消息超时") from exc
            if not data:
                raise ConnectionError("对端已断开")
            self.buffer.extend(data)


def velocity_fields(message):
    output = []
    for key, limit in (("vx_mm_s", 150), ("vy_mm_s", 150), ("wz_mrad_s", 600)):
        value = message.get(key)
        # bool 是 int 的子类，这里明确拒绝 true/false 作为速度。
        if type(value) is not int or abs(value) > limit:
            raise ProtocolError(key + " 必须是范围内整数")
        output.append(value)
    return output

