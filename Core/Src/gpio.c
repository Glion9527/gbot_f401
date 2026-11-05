/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
/* GPIO 是普通数字输入/输出；这里主要配置方向、总使能和板载按键。
 * PC0/PC1 -> A/左前的 AIN1/AIN2；PC2/PC3 -> B/左后的 BIN1/BIN2；
 * PC4/PC5 -> C/右后的 CIN1/CIN2；PC6/PC7 -> D/右前的 DIN1/DIN2。
 * 每台电机的两个 IN 引脚组合决定驱动方向，速度由 TIM1 的 PWM 调节。
 * PA4=STBY 是四路共享的待机控制，拉低时撤销四路驱动。
 * PA5=LD2 是开发板上的绿色用户 LED，不是电机控制输出。
 *
 * PC13=B1 是蓝色 USER 按钮，按下为低电平，松开为高电平。
 * CubeMX 仍将它配置为下降沿 EXTI 输入，但这里未启用对应的 EXTI NVIC，
 * robot_app.c 实际通过反复读取引脚电平完成按键检测和软件消抖。
 * PWM 与编码器引脚使用定时器复用功能，具体配置在 tim.c 中。
 */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/* 初始化先写入低电平，再切换为推挽输出，避免先切换模式后才设初值。
 * 推挽输出可主动输出高/低电平；NoPull 表示不开内部上拉/下拉。
 * GPIO_SPEED_FREQ_LOW 是输出边沿速度档位，不是 PWM 频率或程序运行速度。
 * GPIO 时钟使能只是允许访问该端口，不表示该端口的引脚会自动输出高电平。 */
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin
                          |CIN1_Pin|CIN2_Pin|DIN1_Pin|DIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STBY_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin BIN1_Pin BIN2_Pin
                           CIN1_Pin CIN2_Pin DIN1_Pin DIN2_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin
                          |CIN1_Pin|CIN2_Pin|DIN1_Pin|DIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STBY_Pin LD2_Pin */
  GPIO_InitStruct.Pin = STBY_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
