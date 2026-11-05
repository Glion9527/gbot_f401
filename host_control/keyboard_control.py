#!/usr/bin/env python3
"""终端点按遥控：200ms 没有新按键则发零速，不假装终端提供 key-up。"""
import argparse
import os
import select
import sys
import termios
import time
import tty

from robot_link.client import RobotClient, RobotLinkError, add_client_arguments, token_from_arguments


def main():
    parser = argparse.ArgumentParser(description="低速点按遥控：r 使能，W/S 前后，A/D 横移，Q/E 左右转，0 零速，空格停止解除使能")
    add_client_arguments(parser)
    parser.add_argument("--speed", type=int, default=80, help="平移速度 mm/s，默认80，上限150")
    parser.add_argument("--turn", type=int, default=250, help="转向角速度 mrad/s，默认250，上限600")
    args = parser.parse_args()
    if not 1 <= args.speed <= 150 or not 1 <= args.turn <= 600:
        parser.error("speed 需为1..150，turn需为1..600")
    if not sys.stdin.isatty():
        parser.error("请在有键盘输入的交互终端中运行")
    directions = {"w": (args.speed, 0, 0), "s": (-args.speed, 0, 0),
                  "a": (0, args.speed, 0), "d": (0, -args.speed, 0),
                  "q": (0, 0, args.turn), "e": (0, 0, -args.turn)}
    client, original = None, None
    try:
        client = RobotClient(args.host, args.port, token_from_arguments(args))
        client.connect()
        print("已连接，电机未使能。r使能；W/S前后 A/D横移 Q/E转向；0零速；空格停止解除使能；Ctrl+C退出。")
        print("点按产生最多200ms的小步动作；持续按住依赖系统按键重复，开始重复前可能短暂停顿。反向前先0并等停稳。", flush=True)
        original = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        armed = False
        velocity = (0, 0, 0)
        last_key = 0.0
        pending = None
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], max(0.0, next_send - now))
            if readable:
                keys = os.read(sys.stdin.fileno(), 32).decode("ascii", errors="ignore").lower()
                for key in keys:
                    if key == "\x03":
                        raise KeyboardInterrupt
                    if key == " ":
                        pending, velocity = "stop", (0, 0, 0)
                    # 同批输入或本次发送前已出现空格，停止请求不能被使能键覆盖。
                    elif key == "r" and pending != "stop":
                        pending, velocity = "arm", (0, 0, 0)
                        last_key = 0.0
                    elif key == "0":
                        velocity = (0, 0, 0)
                    elif key in directions and armed and pending != "stop":
                        velocity, last_key = directions[key], time.monotonic()
            now = time.monotonic()
            if now < next_send:
                continue
            next_send = now + 0.05
            if pending is not None:
                kind, pending = pending, None
                reply = client.request(kind)
                armed = kind == "arm" and reply["ok"]
                velocity = (0, 0, 0)
                print("使能成功，等待方向键。" if armed else "已停止/未使能：" + reply["reason"], flush=True)
            elif armed:
                if now - last_key >= 0.2:
                    velocity = (0, 0, 0)
                reply = client.request("velocity", vx_mm_s=velocity[0], vy_mm_s=velocity[1], wz_mrad_s=velocity[2])
                if not reply["ok"]:
                    armed, velocity = False, (0, 0, 0)
                    print("已解除使能：%s；排查并停稳后重新按r。" % reply["reason"], flush=True)
            else:
                client.request("status")
    except KeyboardInterrupt:
        print("\n退出遥控。", flush=True)
    except RobotLinkError as exc:
        print("\n链路停止：%s；不会自动重连或恢复运动。" % exc, file=sys.stderr)
        return 1
    finally:
        if original is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original)
        if client is not None:
            client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
