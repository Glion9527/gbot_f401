/*
 * motor.c
 *
 *  Created on: 2026年10月26日
 *      Author: glion9527
 */

#include "motor.h"
#include "tim.h"
#include <math.h>

/* 检查参数设置 */
#if MOTOR_PWM_LIMIT < 1 || MOTOR_PWM_LIMIT > 1000
#error "MOTOR_PWM_LIMIT must be in 1..1000"
#endif

#if MOTOR_ENCODER_SIGN != 1 && MOTOR_ENCODER_SIGN != -1
#error "MOTOR_ENCODER_SIGN must be +1 or -1"
#endif

#define MODE_STOP   0U
#define MODE_OPEN   1U
#define MODE_SPEED  2U

/* 控制周期：10ms */
#define SAMPLE_S    0.01f

static volatile MotorTelemetry s_motor;
static volatile int16_t s_open_pwm;
static volatile float s_integral;
static volatile uint8_t s_quiet_ticks;

static uint16_t s_encoder_last;
static int8_t s_command_dir;
static int8_t s_hw_dir;

/* 保存中断状态，并进入短临界区 */
static uint32_t Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

/* 恢复原来的中断状态 */
static void Unlock(uint32_t key)
{
    __set_PRIMASK(key);
}

/* 数值限幅 */
static float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* 实际设置方向、PWM和STBY */
static void ApplyPWM(int16_t pwm)
{
    if (pwm > MOTOR_PWM_LIMIT) {
        pwm = MOTOR_PWM_LIMIT;
    }

    if (pwm < -MOTOR_PWM_LIMIT) {
        pwm = -MOTOR_PWM_LIMIT;
    }

    if (pwm == 0) {
        /* STBY低：关闭驱动，电机自由滑行 */
        HAL_GPIO_WritePin(STBY_GPIO_Port,
                          STBY_Pin,
                          GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

        HAL_GPIO_WritePin(AIN1_GPIO_Port,
                          AIN1_Pin,
                          GPIO_PIN_RESET);

        HAL_GPIO_WritePin(AIN2_GPIO_Port,
                          AIN2_Pin,
                          GPIO_PIN_RESET);

        s_hw_dir = 0;
    } else {
        int8_t dir = pwm > 0 ? 1 : -1;

        if (dir != s_hw_dir) {
            /* 设置方向期间先关闭驱动 */
            HAL_GPIO_WritePin(STBY_GPIO_Port,
                              STBY_Pin,
                              GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                AIN1_GPIO_Port,
                AIN1_Pin,
                dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET
            );

            HAL_GPIO_WritePin(
                AIN2_GPIO_Port,
                AIN2_Pin,
                dir > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET
            );

            s_hw_dir = dir;
        }

        uint32_t mag = (uint32_t)(pwm > 0 ? pwm : -pwm);

        /* ARR=4199时，一个PWM周期为4200个计数 */
        uint32_t period =
            __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

        /* 千分比占空比换算成CCR */
        __HAL_TIM_SET_COMPARE(
            &htim1,
            TIM_CHANNEL_1,
            period * mag / 1000U
        );

        HAL_GPIO_WritePin(STBY_GPIO_Port,
                          STBY_Pin,
                          GPIO_PIN_SET);
    }

    s_motor.pwm = pwm;
}

/* 初始化电机模块，TIM4需要在此函数之后启动 */
HAL_StatusTypeDef Motor_Init(void)
{
    s_motor = (MotorTelemetry){0};

    s_open_pwm = 0;
    s_integral = 0.0f;
    s_quiet_ticks = 0;

    s_encoder_last = 0;
    s_command_dir = 0;
    s_hw_dir = 0;

    ApplyPWM(0);

    /* 启动TIM1的PWM输出，初始占空比为0 */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)
        != HAL_OK) {
        return HAL_ERROR;
    }

    /* 清零并启动TIM3硬件编码器计数 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL)
        != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* 停止驱动，保留编码器累计位置 */
void Motor_Stop(void)
{
    uint32_t key = Lock();

    s_motor.mode = MODE_STOP;
    s_motor.target_rpm = 0.0f;

    s_open_pwm = 0;
    s_integral = 0.0f;
    s_quiet_ticks = 0;

    ApplyPWM(0);

    Unlock(key);
}

/* 换向前要求停止，并连续200ms没有编码器计数 */
static bool DirectionAllowed(int8_t dir)
{
    return s_command_dir == 0 ||
           dir == s_command_dir ||
           (s_motor.mode == MODE_STOP &&
            s_quiet_ticks >= 20U);
}

/* 开环控制：正数正转，负数反转 */
bool Motor_SetOpenLoop(int16_t pwm)
{
    if (pwm == 0) {
        Motor_Stop();
        return true;
    }

    if (pwm > MOTOR_PWM_LIMIT) {
        pwm = MOTOR_PWM_LIMIT;
    }

    if (pwm < -MOTOR_PWM_LIMIT) {
        pwm = -MOTOR_PWM_LIMIT;
    }

    int8_t dir = pwm > 0 ? 1 : -1;
    uint32_t key = Lock();

    if (!DirectionAllowed(dir)) {
        Unlock(key);
        return false;
    }

    s_integral = 0.0f;
    s_command_dir = dir;
    s_open_pwm = pwm;

    s_motor.target_rpm = 0.0f;
    s_motor.mode = MODE_OPEN;

    Unlock(key);

    return true;
}

/* 设置闭环目标转速 */
bool Motor_SetSpeedRPM(float rpm)
{
    if (!isfinite(rpm)) {
        return false;
    }

    if (rpm > -0.01f && rpm < 0.01f) {
        Motor_Stop();
        return true;
    }

    int8_t dir = rpm > 0.0f ? 1 : -1;
    uint32_t key = Lock();

    if (!DirectionAllowed(dir)) {
        Unlock(key);
        return false;
    }

    /* 切换模式或方向时清零积分 */
    if (s_motor.mode != MODE_SPEED ||
        dir != s_command_dir) {
        s_integral = 0.0f;
    }

    s_command_dir = dir;
    s_motor.target_rpm = rpm;
    s_motor.mode = MODE_SPEED;

    Unlock(key);

    return true;
}

/* 每10ms调用一次 */
void Motor_ControlTick10ms(void)
{
    /* 读取编码器，使用模差处理16位计数器回绕 */
    uint16_t now =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    uint16_t diff = (uint16_t)(now - s_encoder_last);
    s_encoder_last = now;

    int32_t delta = diff < 32768U
                  ? (int32_t)diff
                  : (int32_t)diff - 65536;

    delta *= MOTOR_ENCODER_SIGN;

    s_motor.delta = delta;
    s_motor.position += delta;

    /* 输出轴转速，单位rpm */
    s_motor.rpm_raw =
        (float)delta * 60.0f /
        (MOTOR_ENCODER_CPR * SAMPLE_S);

    /* 一阶低通滤波 */
    s_motor.rpm +=
        0.25f * (s_motor.rpm_raw - s_motor.rpm);

    /* 记录连续无计数的时间 */
    if (delta == 0) {
        if (s_quiet_ticks < 100U) {
            ++s_quiet_ticks;
        }
    } else {
        s_quiet_ticks = 0;
    }

    if (s_motor.mode == MODE_STOP) {
        ApplyPWM(0);
        return;
    }

    if (s_motor.mode == MODE_OPEN) {
        ApplyPWM(s_open_pwm);
        return;
    }

    /* 以下为转速PI控制 */
    float target = s_motor.target_rpm;

    int8_t dir = target > 0.0f ? 1 : -1;

    float magnitude =
        target > 0.0f ? target : -target;

    float error =
        magnitude - (float)dir * s_motor.rpm;

    float p = MOTOR_KP * error;
    float current = p + s_integral;

    /* 积分抗饱和 */
    if ((current > 0.0f &&
         current < MOTOR_PWM_LIMIT) ||
        (current >= MOTOR_PWM_LIMIT &&
         error < 0.0f) ||
        (current <= 0.0f &&
         error > 0.0f)) {

        s_integral = Clamp(
            s_integral + MOTOR_KI * SAMPLE_S * error,
            0.0f,
            (float)MOTOR_PWM_LIMIT
        );
    }

    /* 固定本次运行方向，控制输出幅值 */
    float u = Clamp(
        p + s_integral,
        0.0f,
        (float)MOTOR_PWM_LIMIT
    );

    ApplyPWM((int16_t)((float)dir * u));
}

/* 在短临界区内复制运行数据 */
void Motor_GetTelemetry(MotorTelemetry *out)
{
    if (out == 0) {
        return;
    }

    uint32_t key = Lock();

    *out = s_motor;

    Unlock(key);
}
