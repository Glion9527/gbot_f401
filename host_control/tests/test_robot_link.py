"""纯软件协议测试：假串口和 socketpair，不打开真实开发板或 LAN 监听端口。"""
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_link.client import RobotClient, RobotLinkError
from robot_link.gateway import Gateway, SerialChannel, SerialLinkError, load_or_create_token
from robot_link.protocol import (JsonLineReader, ProtocolError, crc16, decode_json,
                                 encode_json, parse_serial, serial_frame, serial_payload, velocity_fields)


TOKEN = "test-fixture-" + "x" * 40


# 用内存缓冲和条件变量模拟串口，write 收到控制帧后按测试场景产生 ACK/T 回复。
# 测试可以故意丢回复、延迟遥测、换错 session，核对生产代码如何处理异常。
class FakeSerial:
    def __init__(self):
        self.is_open = True
        self._condition = threading.Condition()
        self._data = bytearray()
        self.commands = []
        self.session = self.seq = 0
        self.armed = False
        self.fault = 0
        self.ignore = False
        self.skip_telem = False
        self.delay_arm = 0.0
        self.wrong_ack = False
        self.result = "OK"

    def read(self, count):
        with self._condition:
            if not self._data and self.is_open:
                self._condition.wait(0.01)
            result = bytes(self._data[:count])
            del self._data[:count]
            return result

    def emit(self, data):
        with self._condition:
            self._data.extend(data)
            self._condition.notify_all()

    def telemetry(self, session=None):
        values = ["T", self.session if session is None else session, 1234, int(self.armed), self.seq, self.fault]
        values += [(1 << 35), -2, 3, 4] + [10, 20, 30, 40] + [0, 0, 0, 0] + [0, 0, 0, 0]
        return serial_frame(",".join(map(str, values)))

    def write(self, data):
        written = len(data)
        fields = serial_payload(data).split(",")
        self.commands.append(fields)
        if self.ignore:
            return len(data)
        code, session, seq = fields[0], int(fields[1]), int(fields[2])
        if code == "H":
            self.session, self.seq, self.armed = session, 0, False
        else:
            self.seq = seq
            if code == "A":
                self.armed = self.result == "OK"
            if code == "S":
                self.armed = False
        ack_session = session + 1 if self.wrong_ack else session
        self.emit(serial_frame("ACK,%d,%d,%s" % (ack_session, seq, self.result)))
        if not self.skip_telem:
            data = self.telemetry()
            if code == "A" and self.delay_arm:
                timer = threading.Timer(self.delay_arm, self.emit, args=(data,))
                timer.daemon = True
                timer.start()
            else:
                self.emit(data)
        return written

    def close(self):
        self.is_open = False
        with self._condition:
            self._condition.notify_all()


# 编码/解析层：检查 CRC、字段边界、JSON/TCP 分片和总超时。
class ProtocolTests(unittest.TestCase):
    def test_crc_vector_and_bad_frame(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        self.assertEqual(serial_payload(serial_frame("H,123,0")), "H,123,0")
        self.assertEqual(serial_payload(serial_frame("H,123,0").replace(b"\n", b"\r\n")), "H,123,0")
        for data in (b"@H,123,0*0000\n", b"H,123,0\n", b"@abc*ZZZZ\n", b"@" + b"a" * 512):
            with self.assertRaises(ProtocolError):
                serial_payload(data)

    def test_telemetry_64bit_order_and_bounds(self):
        serial = FakeSerial()
        telemetry = parse_serial(serial.telemetry())
        self.assertEqual([w["id"] for w in telemetry["wheels"]], ["FL", "FR", "RL", "RR"])
        self.assertEqual(telemetry["wheels"][0]["count"], 1 << 35)
        self.assertEqual(telemetry["wheels"][1]["count"], -2)
        with self.assertRaises(ProtocolError):
            parse_serial(serial_frame("ACK,1,1,MAYBE"))
        with self.assertRaises(ProtocolError):
            parse_serial(serial_frame("T,1,1,0,0,0"))

    def test_json_validation(self):
        for data in (b"[]", b'{"type":"x","type":"y"}', b'{"v":NaN}', b"{" + b"a" * 4096,
                     b"[" * 2000 + b"]" * 2000):
            with self.assertRaises(ProtocolError):
                decode_json(data)
        for value in (True, 1.5, "2", 301):
            with self.assertRaises(ProtocolError):
                velocity_fields({"vx_mm_s": value, "vy_mm_s": 0, "wz_mrad_s": 0})

    def test_network_fragmentation_and_length(self):
        a, b = socket.socketpair()
        try:
            a.sendall(b'{"type":')
            timer = threading.Timer(0.01, a.sendall, args=(b'"status"}\n',))
            timer.start()
            self.assertEqual(JsonLineReader().receive(b, 0.1, time.monotonic), {"type": "status"})
            timer.join()
            a.sendall(b"x" * 4097 + b"\n")
            with self.assertRaises(ProtocolError):
                JsonLineReader().receive(b, 0.1, time.monotonic)
        finally:
            a.close()
            b.close()

    def test_partial_frame_timeout_is_total_deadline(self):
        a, b = socket.socketpair()
        try:
            a.sendall(b"{")
            start = time.monotonic()
            with self.assertRaises(TimeoutError):
                JsonLineReader().receive(b, 0.025, time.monotonic)
            self.assertLess(time.monotonic() - start, 0.1)
        finally:
            a.close()
            b.close()

    def test_token_file_private_and_persistent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "local" / "token"
            token = load_or_create_token(path)
            self.assertEqual(token, load_or_create_token(path))
            self.assertEqual(len(token), 64)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)


# 串口会话层：命令成功必须得到对应确认和有效遥测，不能只看 write 成功。
class SerialTests(unittest.TestCase):
    def setUp(self):
        self.serial = FakeSerial()
        self.channel = SerialChannel(self.serial, command_timeout=0.07)
        self.channel.start()
        self.channel.begin_session()

    def tearDown(self):
        self.channel.close()

    def test_start_is_disarmed_no_velocity_replay(self):
        self.assertFalse(self.channel.telemetry()["armed"])
        time.sleep(0.03)
        self.assertEqual([row[0] for row in self.serial.commands], ["H"])

    def test_arm_waits_for_new_telemetry(self):
        self.serial.delay_arm = 0.03
        start = time.monotonic()
        result = self.channel.command("arm")
        self.assertTrue(result["ok"])
        self.assertTrue(result["telemetry"]["armed"])
        self.assertGreaterEqual(time.monotonic() - start, 0.025)

    def test_arm_ack_without_new_telemetry_fails(self):
        self.serial.skip_telem = True
        with self.assertRaises(SerialLinkError):
            self.channel.command("arm")

    def test_wrong_session_ack_is_not_accepted(self):
        self.serial.wrong_ack = True
        with self.assertRaises(SerialLinkError):
            self.channel.command("arm")

    def test_old_session_telemetry_does_not_refresh_age(self):
        with self.channel._condition:
            self.channel._telemetry_at = time.monotonic() - 1
        self.serial.emit(self.serial.telemetry(self.channel.session + 1))
        time.sleep(0.02)
        with self.assertRaises(SerialLinkError):
            self.channel.telemetry()

    def test_refused_arm_is_false(self):
        self.serial.result = "BUSY"
        self.assertEqual(self.channel.command("arm")["reason"], "BUSY")

    def test_fault_telemetry_cannot_confirm_arm(self):
        self.serial.fault = 1
        self.assertFalse(self.channel.command("arm")["ok"])

    def test_velocity_is_not_repeated_and_seq_can_wrap(self):
        self.assertTrue(self.channel.command("arm")["ok"])
        self.channel.seq = 0xFFFFFFFF
        self.assertTrue(self.channel.command("velocity", vx_mm_s=50, vy_mm_s=0, wz_mrad_s=0)["ok"])
        self.assertEqual(self.serial.commands[-1][2], "0")
        time.sleep(0.02)
        self.assertEqual(sum(cmd[0] == "V" for cmd in self.serial.commands), 1)

    def test_serial_corruption_closes_control_path(self):
        self.serial.emit(b"@T,broken*0000\n")
        time.sleep(0.02)
        with self.assertRaises(SerialLinkError):
            self.channel.command("arm")


# 网关层：通过 socketpair 模拟客户端连接，验证认证、请求和退出处理。
class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.serial = FakeSerial()
        self.channel = SerialChannel(self.serial, command_timeout=0.07)
        self.channel.start()
        self.gateway = Gateway(self.channel, TOKEN, challenge_ttl=0.08)
        self.a, self.b = socket.socketpair()
        self.reader = JsonLineReader()
        self.thread = threading.Thread(target=self.gateway.serve_client, args=(self.b,))
        self.thread.start()

    def tearDown(self):
        self.a.close()
        self.thread.join(timeout=0.3)
        self.channel.close()
        self.assertFalse(self.thread.is_alive())

    def receive(self):
        return self.reader.receive(self.a, 0.3, time.monotonic)

    def authenticate(self):
        self.a.sendall(encode_json({"type": "auth", "token": TOKEN}))
        welcome = self.receive()
        self.assertEqual(welcome["type"], "welcome")
        self.assertFalse(welcome["telemetry"]["armed"])
        return welcome["challenge"]

    def test_authentication_rejects_wrong_and_unicode_tokens(self):
        self.a.sendall(encode_json({"type": "auth", "token": "无效"}))
        self.assertEqual(self.receive()["type"], "error")
        self.assertEqual(self.serial.commands, [])

    def test_commands_and_disconnect_stop(self):
        challenge = self.authenticate()
        self.a.sendall(encode_json({"type": "arm", "challenge": challenge}))
        reply = self.receive()
        self.assertTrue(reply["ok"])
        self.a.sendall(encode_json({"type": "velocity", "challenge": reply["challenge"],
                                   "vx_mm_s": 50, "vy_mm_s": 0, "wz_mrad_s": 0}))
        self.assertTrue(self.receive()["ok"])
        self.a.shutdown(socket.SHUT_RDWR)
        self.thread.join(timeout=0.2)
        self.assertEqual(self.serial.commands[-1][0], "S")
        self.assertEqual(sum(cmd[0] == "V" for cmd in self.serial.commands), 1)

    def test_status_does_not_send_velocity_or_arm(self):
        challenge = self.authenticate()
        for _ in range(3):
            self.a.sendall(encode_json({"type": "status", "challenge": challenge}))
            reply = self.receive()
            self.assertTrue(reply["ok"])
            challenge = reply["challenge"]
        self.assertEqual([cmd[0] for cmd in self.serial.commands], ["H"])

    def test_challenge_reuse_stops_without_second_arm(self):
        challenge = self.authenticate()
        message = encode_json({"type": "arm", "challenge": challenge})
        self.a.sendall(message + message)
        self.assertEqual(self.receive()["type"], "reply")
        self.assertEqual(self.receive()["type"], "error")
        self.assertEqual(sum(cmd[0] == "A" for cmd in self.serial.commands), 1)
        self.assertEqual(self.serial.commands[-1][0], "S")

    def test_expired_challenge_never_arms(self):
        self.authenticate()
        self.assertEqual(self.receive()["type"], "error")
        self.assertNotIn("A", [cmd[0] for cmd in self.serial.commands])

    def test_invalid_velocity_stops_and_closes(self):
        challenge = self.authenticate()
        self.a.sendall(encode_json({"type": "velocity", "challenge": challenge,
                                   "vx_mm_s": True, "vy_mm_s": 0, "wz_mrad_s": 0}))
        self.assertEqual(self.receive()["type"], "error")
        self.assertNotIn("V", [cmd[0] for cmd in self.serial.commands])

    def test_unknown_request_and_overlong_line_stop(self):
        self.authenticate()
        self.a.sendall(b"x" * 4097 + b"\n")
        self.assertEqual(self.receive()["type"], "error")
        self.assertEqual(self.serial.commands[-1][0], "S")


# 客户端层：响应错误后关闭连接，避免重发可能已被固件执行过的运动请求。
class ClientTests(unittest.TestCase):
    def test_bad_response_aborts_without_retry(self):
        fake_socket = mock.Mock()
        fake_socket.recv.side_effect = [encode_json({"type": "welcome", "version": 1, "challenge": "a" * 32}),
                                        TimeoutError("测试超时")]
        with mock.patch("robot_link.client.socket.create_connection", return_value=fake_socket):
            client = RobotClient("example", 8765, TOKEN)
            client.connect()
            with self.assertRaises(RobotLinkError):
                client.request("velocity", vx_mm_s=50, vy_mm_s=0, wz_mrad_s=0)
            self.assertEqual(fake_socket.sendall.call_count, 2)
            self.assertTrue(fake_socket.close.called)
            with self.assertRaises(RobotLinkError):
                client.request("arm")


# 回归场景：拒绝命令后的 STOP 必须顺序确认，防止清队列影响紧随其后的帧。
class QueuedStopTests(unittest.TestCase):
    """模拟真实MCU的STOP清RX：停止确认前发第二帧，会被丢弃。"""
    def test_rejected_velocity_then_client_stop_are_serialized(self):
        serial = FakeSerial()
        original_write = serial.write
        stop_pending = threading.Event()
        overlap = []
        timers = []

        def delayed_write(data):
            fields = serial_payload(data).split(',')
            if stop_pending.is_set():
                overlap.append(fields[0])
                return len(data)
            if fields[0] == 'V':
                serial.result = 'FAULT'
                try:
                    return original_write(data)
                finally:
                    serial.result = 'OK'
            if fields[0] == 'S':
                stop_pending.set()
                def finish():
                    stop_pending.clear()
                    original_write(data)
                timer = threading.Timer(0.02, finish)
                timers.append(timer)
                timer.start()
                return len(data)
            return original_write(data)

        serial.write = delayed_write
        channel = SerialChannel(serial, command_timeout=0.1)
        channel.start()
        a, b = socket.socketpair()
        thread = threading.Thread(target=Gateway(channel, TOKEN).serve_client, args=(b,))
        thread.start()
        reader = JsonLineReader()
        try:
            a.sendall(encode_json(dict(type='auth', token=TOKEN)))
            welcome = reader.receive(a, 0.3, time.monotonic)
            a.sendall(encode_json(dict(type='velocity', challenge=welcome['challenge'],
                                      vx_mm_s=50, vy_mm_s=0, wz_mrad_s=0)))
            rejected = reader.receive(a, 0.3, time.monotonic)
            self.assertEqual(rejected['reason'], 'FAULT')
            self.assertFalse(rejected['telemetry']['armed'])
            a.sendall(encode_json(dict(type='stop', challenge=rejected['challenge'])))
            stopped = reader.receive(a, 0.3, time.monotonic)
            self.assertTrue(stopped['ok'])
            a.sendall(encode_json(dict(type='status', challenge=stopped['challenge'])))
            self.assertTrue(reader.receive(a, 0.3, time.monotonic)['ok'])
            self.assertEqual(overlap, [])
        finally:
            a.close()
            thread.join(timeout=0.5)
            for timer in timers:
                timer.join(timeout=0.2)
            self.assertFalse(thread.is_alive())
            channel.close()

    def test_error_cleanup_sends_one_confirmed_stop(self):
        serial = FakeSerial()
        channel = SerialChannel(serial, command_timeout=0.1)
        channel.start()
        a, b = socket.socketpair()
        thread = threading.Thread(target=Gateway(channel, TOKEN).serve_client, args=(b,))
        thread.start()
        reader = JsonLineReader()
        try:
            a.sendall(encode_json(dict(type='auth', token=TOKEN)))
            welcome = reader.receive(a, 0.3, time.monotonic)
            a.sendall(encode_json(dict(type='unknown', challenge=welcome['challenge'])))
            self.assertEqual(reader.receive(a, 0.3, time.monotonic)['type'], 'error')
            thread.join(timeout=0.3)
            self.assertFalse(thread.is_alive())
            self.assertEqual(sum(row[0]=='S' for row in serial.commands), 1)
            # 清理已确认，立即新会话不会被上一条STOP清掉。
            self.assertFalse(channel.begin_session()['armed'])
        finally:
            a.close()
            thread.join(timeout=0.3)
            channel.close()


if __name__ == "__main__":
    unittest.main()
