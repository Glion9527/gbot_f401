#!/usr/bin/env python3
"""Mac 入口：只有显式运行本脚本才会打开开发板串口。"""
import argparse
from pathlib import Path
import sys

from robot_link.gateway import Gateway, SerialChannel, load_or_create_token


def main():
    parser = argparse.ArgumentParser(description="Mac 串口到 Ubuntu TCP 中转（上电、连接均不自动运动）")
    parser.add_argument("--serial", required=True, help="开发板串口，例如 /dev/cu.usbmodem1103")
    parser.add_argument("--bind", default="127.0.0.1", help="监听地址；Ubuntu 访问时显式用 0.0.0.0 或 Mac 的 VM 网络地址")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--token-file", type=Path, default=Path(__file__).parent / ".local" / "token")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        parser.error("请先在 Python 虚拟环境中安装：python3 -m pip install -r host_control/requirements.txt")
    channel = None
    try:
        token = load_or_create_token(args.token_file)
        # exclusive 避免与 screen、其他中转进程同时占用；DTR/RTS 不充当运动许可。
        port = serial.Serial(args.serial, baudrate=115200, timeout=0.02, write_timeout=0.05,
                             rtscts=False, dsrdtr=False, exclusive=True)
        channel = SerialChannel(port)
        channel.start()
        # 启动先验证固件协议，并明确停止；旧模式固件不会被当作遥控固件使用。
        channel.begin_session()
        print("串口握手成功，底盘未使能。配对码文件：%s" % args.token_file, flush=True)
        print("配对码（只复制到自己的 Ubuntu）：%s" % token, flush=True)
        Gateway(channel, token).serve(args.bind, args.port)
    except KeyboardInterrupt:
        print("\n退出中转，撤销驱动。", flush=True)
    except Exception as exc:
        print("中转停止：%s" % exc, file=sys.stderr)
        return 1
    finally:
        if channel is not None:
            channel.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
