/*
 * motor.h
 *
 *  Created on: 2026年10月26日
 *      Author: glion9527
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * 编码器四倍频后，减速箱输出轴每转的计数。
 * 1320只是示例，确认电机参数后必须修改。
 */
#define MOTOR_ENCODER_CPR   1320.0f

/* 正向驱动应测得正转速；方向相反时改成 -1 */
#define MOTOR_ENCODER_SIGN  (+1)

/* PWM千分比上限：300表示30%，这不是电流限制 */
#define MOTOR_PWM_LIMIT     300

/* PI教学起始参数，后面根据实际电机调整 */
#define MOTOR_KP            2.0f
#define MOTOR_KI            10.0f

/* 电机运行数据 */
typedef struct {
    float rpm_raw;       /* 原始转速，单位 rpm */
    float rpm;           /* 滤波后转速，单位 rpm */
    float target_rpm;    /* 目标转速，单位 rpm */
    int64_t position;    /* 编码器累计计数 */
    int32_t delta;       /* 最近10ms的计数变化 */
    int16_t pwm;         /* 带方向的PWM千分比 */
    uint8_t mode;        /* 0停止，1开环，2转速闭环 */
} MotorTelemetry;

/* 初始化电机模块 */
HAL_StatusTypeDef Motor_Init(void);

/* 停止驱动，电机自由滑行 */
void Motor_Stop(void);

/* 设置开环PWM：正数正转，负数反转 */
bool Motor_SetOpenLoop(int16_t pwm);

/* 设置减速箱输出轴的目标转速 */
bool Motor_SetSpeedRPM(float rpm);

/* 每10ms调用一次，后面放进TIM4中断回调 */
void Motor_ControlTick10ms(void);

/* 获取一份电机运行数据 */
void Motor_GetTelemetry(MotorTelemetry *out);

#endif
