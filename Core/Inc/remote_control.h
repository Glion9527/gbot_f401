#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#include "usart.h"

/* Mac/ROS 2 只下发车身速度；四轮调速仍由原来的 motor4 执行。
 * 串口帧、单位和停机条件见 docs/remote-control-protocol.md。
 * Init 在 Motor4_Init 后调用；只有 ROBOT_APP_MODE=7 才启动接收。 */
/* 初始化返回HAL_OK仅表示接收已挂起；还要经过H握手、A使能、V速度指令才会运动。
 * 本模块拥有USART2遥控帧的接收状态，普通上位机文本不会被当成运动命令。 */
HAL_StatusTypeDef RemoteControl_Init(void);
/* 在main的while(1)中持续调用：把已收到的字节拼成命令，并约每100ms回传四轮数据。
 * Poll本身不等待下一帧到达，串口同步发送期间仍可被控制定时中断抢占。 */
void RemoteControl_Poll(void);

/* TIM9 中断中先调用此函数，再调用 Motor4_ControlTick10ms。
 * 即使主循环卡在串口发送，也能检查指令失效及蓝色按钮。 */
void RemoteControl_Tick10ms(void);

/* 由 usart.c 的 HAL 回调转发；中断只收字节/撤销许可，不解析或打印。 */
/* 参数uart用于确认中断来源是huart2，其他串口的回调不会改变本模块状态。 */
void RemoteControl_UART_RxCplt(UART_HandleTypeDef *uart);
void RemoteControl_UART_Error(UART_HandleTypeDef *uart);

#endif
