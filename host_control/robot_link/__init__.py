"""Mac 与 Ubuntu 共用的底盘接口，不依赖 ROS 2。"""
# client：Ubuntu/上位机的 TCP 客户端；gateway：Mac 端 TCP 与串口的衔接。
# protocol：两端共用的字段校验、JSON 编解码和串口 CRC 帧定义。
# ROS 2 节点也调用这里的 client，因此 ROS 话题与串口之间可以复用同一通信实现。

