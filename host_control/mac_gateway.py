#!/usr/bin/env python3
"""Mac 入口：只有显式运行本脚本才会打开开发板串口。"""
# 整条链路：Ubuntu 程序 -> TCP/JSON -> 本网关 -> USB 串口协议 -> STM32。
# 本文件负责组装和启动；收发、握手、协议转换的实现位于 robot_link/gateway.py。
# 电机的编码器测速和 PI 调速仍由 STM32 执行，本网关传递运动目标与反馈。
import argparse
from pathlib import Path
import sys

from robot_link.gateway import Gateway, SerialChannel, load_or_create_token


def main():
    # --serial 指向 Mac 上真实的开发板设备；--bind/--port 决定网络客户端接入哪里。
    # 127.0.0.1 只接受本机连接，Ubuntu 从另一台主机/虚拟机访问时要显式改监听地址。
    parser = argparse.ArgumentParser(description="Mac 串口到 Ubuntu TCP 中转（上电、连接均不自动运动）")
    parser.add_argument("--serial", required=True, help="开发板串口，例如 /dev/cu.usbmodem1103")
    parser.add_argument("--bind", default="127.0.0.1", help="监听地址；Ubuntu 访问时显式用 0.0.0.0 或 Mac 的 VM 网络地址")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--token-file", type=Path, default=Path(__file__).parent / ".local" / "token")
    args = parser.parse_args()
    # 延迟导入 pyserial：只有启动入口才需要这个依赖，普通协议模块可以独立测试。
    try:
        import serial
    except ImportError:
        parser.error("请先在 Python 虚拟环境中安装：python3 -m pip install -r host_control/requirements.txt")
    channel = None
    try:
        # 首次启动创建配对码，以后复用文件中的值；Ubuntu 认证时需提供相同配对码。
        token = load_or_create_token(args.token_file)
        # exclusive 避免与 screen、其他中转进程同时占用；DTR/RTS 不充当运动许可。
        port = serial.Serial(args.serial, baudrate=115200, timeout=0.02, write_timeout=0.05,
                             rtscts=False, dsrdtr=False, exclusive=True)
        # timeout/write_timeout 的单位是秒，分别约束一次串口读取和写入的等待时间。
        # 后台读取线程持续接收 STM32 的 ACK 与遥测，前台线程负责发命令并等待确认。
        channel = SerialChannel(port)
        channel.start()
        # 启动先验证固件协议，并明确停止；旧模式固件不会被当作遥控固件使用。
        channel.begin_session()
        print("串口握手成功，底盘未使能。配对码文件：%s" % args.token_file, flush=True)
        print("配对码（只复制到自己的 Ubuntu）：%s" % token, flush=True)
        # serve() 会持续监听并处理连接，直到用户退出或发生未处理异常。
        Gateway(channel, token).serve(args.bind, args.port)
    except KeyboardInterrupt:
        print("\n退出中转，撤销驱动。", flush=True)
    except Exception as exc:
        print("中转停止：%s" % exc, file=sys.stderr)
        return 1
    finally:
        # 无论是正常 Ctrl+C 还是异常退出，都走统一清理流程：尝试停止、关闭串口线程。
        if channel is not None:
            channel.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
