/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "robot_app.h"
#if ROBOT_APP_MODE == 7
#include "remote_control.h"
#endif
/* USART2 用于把运行状态发送到电脑：PA2=TX（发送），PA3=RX（接收）。
 * NUCLEO 板默认通过 ST-LINK 虚拟串口转到 USB，电脑串口终端需设为
 * 115200 波特率、8 位数据、无校验、1 位停止位，即常说的 115200 8N1。
 * 波特率表示每秒传输的符号数，不是每秒能发送 115200 个字符。
 * 遥控模式用接收中断把字节存入缓冲，主循环校验并执行命令、发送状态；
 * 原测试模式由 robot_app.c 在主循环中调用 HAL_UART_Transmit 输出状态，
 * TIM9 调速中断不负责打印；串口日志显示的是采样时的状态快照。 */
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* huart2 保存 USART2 的配置和运行状态，由 HAL_UART_Init 应用到硬件。
   * TX_RX 同时允许收发；模式7接收CRC串口运动命令，其余模式发送测试日志。
   * HwFlowCtl=NONE：无需 RTS/CTS 流控线，电脑终端也应关闭硬件流控。 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
  /* 打开 USART2/GPIOA 时钟，再将 PA2、PA3 设为 AF7 复用功能。
   * 引脚交给 USART 硬件收发，不作为普通 GPIO 手动翻转。 */
  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART2_TX_Pin|USART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */
  /* 优先级6低于TIM9的5，接收字节不会抢占10ms电机控制。 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART2_TX_Pin|USART2_RX_Pin);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
#if ROBOT_APP_MODE == 7
    RemoteControl_UART_RxCplt(uart);
#else
    (void)uart;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
#if ROBOT_APP_MODE == 7
    RemoteControl_UART_Error(uart);
#else
    (void)uart;
#endif
}

/* USER CODE END 1 */
