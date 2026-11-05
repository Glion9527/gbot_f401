#!/usr/bin/env python3
"""只检查链路和遥测，不发送 ARM 或非零速度。"""
# 在实际遥控前可用它确认 TCP 认证、Mac 串口握手和 STM32 遥测是否连通。
# status 读取网关缓存中仍有效的遥测，调用频率不等于 STM32 采样或发送频率。
import argparse
import json
import sys
import time

from robot_link.client import RobotClient, RobotLinkError, add_client_arguments, token_from_arguments


def main():
    parser = argparse.ArgumentParser(description="只读链路检查；20Hz 查询，约每秒显示一组遥测，不使能电机")
    add_client_arguments(parser)
    parser.add_argument("--seconds", type=float, default=0, help="运行秒数；0 表示持续到 Ctrl+C")
    args = parser.parse_args()
    client = None
    try:
        client = RobotClient(args.host, args.port, token_from_arguments(args))
        welcome = client.connect()
        print("连接成功，未使能电机。session=%s" % welcome["telemetry"]["session"], flush=True)
        # monotonic 是只用于计时差的单调时钟，不受系统日期/时区调整影响。
        start, last_print = time.monotonic(), 0.0
        while args.seconds <= 0 or time.monotonic() - start < args.seconds:
            # 通信约 20Hz、显示约 1Hz：持续验证链路，同时避免终端每次请求都刷屏。
            due = time.monotonic() + 0.05
            reply = client.request("status")
            if not reply["ok"]:
                raise RobotLinkError(reply["reason"])
            if time.monotonic() - last_print >= 1.0:
                print(json.dumps(reply["telemetry"], ensure_ascii=False), flush=True)
                last_print = time.monotonic()
            time.sleep(max(0.0, due - time.monotonic()))
    except KeyboardInterrupt:
        pass
    except (RobotLinkError, KeyError) as exc:
        print("检查失败：%s" % exc, file=sys.stderr)
        return 1
    finally:
        if client is not None:
            # 探测期间不发运动指令；退出仍使用公共 close()，它会尽力发送一次 STOP。
            client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

