#!/usr/bin/env python3
"""终端点按遥控：200ms 没有新按键则发零速，不假装终端提供 key-up。"""
# 这是不依赖 ROS 2 的网络遥控入口，可在能连接 Mac 网关的终端中运行。
# 它只把键盘映射成车身速度；Mac 负责转换成串口帧，STM32 负责四轮运动计算。
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
    parser.add_argument("--speed", type=int, default=80, help="前后速度 mm/s，默认80，上限300；横移仍限150")
    parser.add_argument("--turn", type=int, default=250, help="转向角速度 mrad/s，默认250，上限600")
    args = parser.parse_args()
    if not 1 <= args.speed <= 300 or not 1 <= args.turn <= 600:
        parser.error("speed 需为1..300，turn需为1..600")
    if not sys.stdin.isatty():
        parser.error("请在有键盘输入的交互终端中运行")
    # 每个元组依次为：前进速度 mm/s、向左速度 mm/s、逆时针角速度 mrad/s。
    # 按键只选择一种运动分量；此终端版不把同时按下的多个方向键合成为斜向运动。
    # 直行可用到300mm/s，横移仍按网关/固件的150mm/s上限截断。
    strafe = min(args.speed, 150)
    directions = {"w": (args.speed, 0, 0), "s": (-args.speed, 0, 0),
                  "a": (0, strafe, 0), "d": (0, -strafe, 0),
                  "q": (0, 0, args.turn), "e": (0, 0, -args.turn)}
    client, original = None, None
    try:
        client = RobotClient(args.host, args.port, token_from_arguments(args))
        client.connect()
        print("已连接，电机未使能。r使能；W/S前后 A/D横移 Q/E转向；0零速；空格停止解除使能；Ctrl+C退出。")
        print("点按产生最多200ms的小步动作；持续按住依赖系统按键重复，开始重复前可能短暂停顿。反向前先0并等停稳。", flush=True)
        # 保存原始终端设置，切到 cbreak 后不必按回车就能读取单个按键。
        # finally 必须恢复这些设置，否则退出程序后终端可能仍不按平常方式响应输入。
        original = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        # armed 是本客户端记录的运行许可；连接成功本身不允许发送运动目标。
        # pending 保存待发送的使能/停止请求，velocity 保存当前三轴目标。
        armed = False
        velocity = (0, 0, 0)
        last_key = 0.0
        pending = None
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            # 同时等键盘与下一次发送时刻，避免空转占满 CPU，也避免等待回车阻塞通信。
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
            # 最快约每 50ms（20Hz）发一次请求；网络请求耗时较长时不补发积压的旧请求。
            next_send = now + 0.05
            if pending is not None:
                kind, pending = pending, None
                reply = client.request(kind)
                # 只有本次 ARM 得到肯定回复才进入运行许可；使能完成后目标仍归零。
                armed = kind == "arm" and reply["ok"]
                velocity = (0, 0, 0)
                print("使能成功，等待方向键。" if armed else "已停止/未使能：" + reply["reason"], flush=True)
            elif armed:
                # 普通终端没有可靠的“松键”事件，用最近一次方向键时间限制每次小步。
                # 零速请求保留使能；空格对应 STOP，请求停止并解除使能。
                if now - last_key >= 0.2:
                    velocity = (0, 0, 0)
                reply = client.request("velocity", vx_mm_s=velocity[0], vy_mm_s=velocity[1], wz_mrad_s=velocity[2])
                if not reply["ok"]:
                    armed, velocity = False, (0, 0, 0)
                    print("已解除使能：%s；排查并停稳后重新按r。" % reply["reason"], flush=True)
            else:
                # 未使能时仍定期查询状态，维持网关的请求/响应会话并检查链路是否正常。
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
            # close() 会尝试发送 STOP，再关闭网络连接，不保存下一次启动的运动目标。
            client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
