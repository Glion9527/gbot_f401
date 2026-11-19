#ifndef LINK_V2_H
#define LINK_V2_H

/*
 * STM32 侧二进制链路 v2 的模块接口。
 *
 * 与 v1 的 remote_control.c 并列：USART2 中断入口、主循环 Poll、TIM9 Tick
 * 由 usart.c / robot_app.c / main.c 通过 LINK_PROTOCOL_V2 选择调用本模块。
 *
 * 协议字段、CRC、帧长全部来自 link_v2_contract.h（仓库根 protocol/link_v2.h），
 * 本文件不重复定义帧格式。
 */

#include "main.h"
#include "usart.h"
#include "link_v2_contract.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：停止四轮、清空状态、启动 USART2 单字节接收中断。 */
HAL_StatusTypeDef LinkV2_Init(void);

/* 主循环调用：解析命令、喂电机看门狗、按 50Hz 发送 TELEM。 */
void LinkV2_Poll(void);

/* TIM9 的 10ms 回调调用：更新运动状态、累计编码器增量、检查 300ms 超时。 */
void LinkV2_Tick10ms(void);

/* 主循环在 IMU 读取成功后调用；sample 为 NULL 表示本帧 IMU 数据无效。
 * 只拷贝量化后的整数，不在链路层做 I2C 访问。 */
typedef struct {
    uint16_t flags;              /* LINK_V2_IMU_* */
    int16_t acc_mg[3];           /* mg，含重力 */
    int16_t gyro_dps10[3];       /* 0.1 deg/s */
    int16_t mag_ut10[3];         /* 0.1 uT */
    int16_t temp_c10;            /* 0.1 degC */
} LinkV2ImuSample;

void LinkV2_UpdateImu(const LinkV2ImuSample *sample);

/* HAL USART2 回调转交（由 usart.c 的条件编译分派）。 */
void LinkV2_UART_RxCplt(UART_HandleTypeDef *uart);
void LinkV2_UART_Error(UART_HandleTypeDef *uart);
void LinkV2_UART_TxCplt(UART_HandleTypeDef *uart);

#ifdef LINK_V2_HOST_TEST
/* 主机仿真测试钩子；不参与固件编译。 */
void LinkV2_TestReset(void);
void LinkV2_TestFeed(const uint8_t *data, size_t len);
bool LinkV2_TestArmed(void);
uint16_t LinkV2_TestLastSeq(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* LINK_V2_H */
