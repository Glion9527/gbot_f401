#include "motor4.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>

/* 手动导入时命名为 Core/Src/motor4.c。
 * 四个编码器均设ARR=65535，单个10ms周期内计数差必须小于32768。
 * TIM9回调每10ms调用本模块，禁止同时调用旧单路Motor_ControlTick10ms。 */
#define SAMPLE_S             0.01f
#define QUIET_TICKS          20U
#define NO_FEEDBACK_TICKS    20U
#define WRONG_DIR_TICKS      10U
#define STBY_PORT            GPIOA
#define STBY_PIN             GPIO_PIN_4

#if MOTOR4_PWM_LIMIT < 1 || MOTOR4_PWM_LIMIT > 1000
#error "MOTOR4_PWM_LIMIT must be in 1..1000"
#endif

typedef struct {
    TIM_HandleTypeDef *encoder;
    uint32_t channel;
    GPIO_TypeDef *in_port;
    uint16_t in1;
    uint16_t in2;
    float cpr;
    float kp;
    float ki;
    int8_t encoder_sign;
    int8_t drive_sign;
} Motor4Config;

/* 逻辑轮序固定FL,FR,RL,RR，对应物理接口A,D,B,C。
 * 各物理接口的引脚不变，本次轮位重映射不需要修改CubeMX。 */
static const Motor4Config s_cfg[MOTOR4_COUNT] = {
    /* A：左前 */
    [MOTOR4_FL] = {
        &htim3, TIM_CHANNEL_1, GPIOC, GPIO_PIN_0, GPIO_PIN_1,
        MOTOR4_FL_CPR, MOTOR4_KP, MOTOR4_KI,
        MOTOR4_FL_ENCODER_SIGN, MOTOR4_FL_DRIVE_SIGN
    },
    /* D：右前 */
    [MOTOR4_FR] = {
        &htim5, TIM_CHANNEL_4, GPIOC, GPIO_PIN_6, GPIO_PIN_7,
        MOTOR4_FR_CPR, MOTOR4_KP, MOTOR4_KI,
        MOTOR4_FR_ENCODER_SIGN, MOTOR4_FR_DRIVE_SIGN
    },
    /* B：左后 */
    [MOTOR4_RL] = {
        &htim2, TIM_CHANNEL_2, GPIOC, GPIO_PIN_2, GPIO_PIN_3,
        MOTOR4_RL_CPR, MOTOR4_KP, MOTOR4_KI,
        MOTOR4_RL_ENCODER_SIGN, MOTOR4_RL_DRIVE_SIGN
    },
    /* C：右后 */
    [MOTOR4_RR] = {
        &htim4, TIM_CHANNEL_3, GPIOC, GPIO_PIN_4, GPIO_PIN_5,
        MOTOR4_RR_CPR, MOTOR4_KP, MOTOR4_KI,
        MOTOR4_RR_ENCODER_SIGN, MOTOR4_RR_DRIVE_SIGN
    }
};

typedef struct {
    int16_t open_pwm;
    float integral;
    uint16_t encoder_last;
    int8_t command_dir;
    uint8_t quiet_ticks;
    uint8_t no_feedback_ticks;
    uint8_t wrong_dir_ticks;
} Motor4Runtime;

static volatile Motor4Telemetry s_motor[MOTOR4_COUNT];
static volatile Motor4Runtime s_rt[MOTOR4_COUNT];
static int8_t s_hw_dir[MOTOR4_COUNT];
static volatile bool s_initialized;
static volatile uint32_t s_watchdog_ms;

static uint32_t Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static void Unlock(uint32_t key)
{
    __set_PRIMASK(key);
}

static bool ValidId(Motor4Id id)
{
    return (unsigned)id < (unsigned)MOTOR4_COUNT;
}

static float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static int16_t ClampPWM(int16_t pwm)
{
    if (pwm > MOTOR4_PWM_LIMIT) return MOTOR4_PWM_LIMIT;
    if (pwm < -MOTOR4_PWM_LIMIT) return -MOTOR4_PWM_LIMIT;
    return pwm;
}

static bool HasFault(void)
{
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (s_motor[i].fault != MOTOR4_FAULT_NONE) return true;
    }
    return false;
}

static bool AnyActive(void)
{
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (s_motor[i].mode != MOTOR4_MODE_STOP) return true;
    }
    return false;
}

/* 调用方位于短临界区，或来自TIM9中断。
 * 本板四路共享STBY，不能每停一台就把STBY拉低并保持。
 * 其他路运行时，本路IN1=IN2=0、PWM持续高，进入TB6612的STOP高阻态。
 * 拓扑变化时先关共享STBY并提交CCR影子寄存器，避免从100%高电平
 * 的单路停止状态启动时，预装载尚未生效造成短暂全占空比。 */
static void CommitOutputs(const int16_t logical[MOTOR4_COUNT])
{
    int16_t pwm[MOTOR4_COUNT];
    int8_t hw_dir[MOTOR4_COUNT];
    bool any = false;
    bool changed = false;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        pwm[i] = ClampPWM(logical[i]);
        hw_dir[i] = pwm[i] == 0 ? 0 :
            (int8_t)((pwm[i] > 0 ? 1 : -1) * s_cfg[i].drive_sign);
        if (pwm[i] != 0) any = true;
        if (hw_dir[i] != s_hw_dir[i]) changed = true;
    }

    if (changed || !any) {
        HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    }

    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (changed || !any) {
            HAL_GPIO_WritePin(s_cfg[i].in_port, s_cfg[i].in1,
                hw_dir[i] > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(s_cfg[i].in_port, s_cfg[i].in2,
                hw_dir[i] < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        uint32_t ccr;
        if (pwm[i] == 0) {
            ccr = any ? period : 0U;
        } else {
            uint32_t magnitude = (uint32_t)(pwm[i] > 0 ? pwm[i] : -pwm[i]);
            ccr = period * magnitude / 1000U;
        }
        __HAL_TIM_SET_COMPARE(&htim1, s_cfg[i].channel, ccr);
        s_hw_dir[i] = hw_dir[i];
        s_motor[i].pwm = pwm[i];
    }

    if (changed) {
        /* STM32F4 HAL没有__HAL_TIM_GENERATE_EVENT宏，直接产生UG。
         * TIM1未启用更新中断/DMA；UG会重置PWM计数相位并装载四个CCR。 */
        htim1.Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    }

    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN,
                     any ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void StopState(unsigned i)
{
    if (s_motor[i].mode != MOTOR4_MODE_STOP) {
        s_rt[i].quiet_ticks = 0;
    }
    s_motor[i].mode = MOTOR4_MODE_STOP;
    s_motor[i].target_rpm = 0.0f;
    s_rt[i].open_pwm = 0;
    s_rt[i].integral = 0.0f;
    s_rt[i].no_feedback_ticks = 0;
    s_rt[i].wrong_dir_ticks = 0;
}

static void StopAllInternal(void)
{
    /* 故障或停机时先撤销硬件使能，再更新软件状态。 */
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) StopState(i);
    const int16_t zero[MOTOR4_COUNT] = {0};
    CommitOutputs(zero);
}

static void LatchFault(unsigned source, uint8_t fault)
{
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    if (source < MOTOR4_COUNT) {
        s_motor[source].fault |= fault;
    } else {
        for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
            s_motor[i].fault |= fault;
        }
    }
    StopAllInternal();
}

static bool DirectionAllowed(unsigned i, int8_t dir)
{
    return s_rt[i].command_dir == 0 ||
           dir == s_rt[i].command_dir ||
           (s_motor[i].mode == MOTOR4_MODE_STOP &&
            s_rt[i].quiet_ticks >= QUIET_TICKS);
}

static bool ValidSpeed(float rpm)
{
    return isfinite(rpm) &&
           rpm >= -MOTOR4_MAX_TARGET_RPM && rpm <= MOTOR4_MAX_TARGET_RPM;
}

static float NormalizeSpeed(float rpm)
{
    return rpm > -0.01f && rpm < 0.01f ? 0.0f : rpm;
}

static void SetSpeedState(unsigned i, float rpm)
{
    if (rpm == 0.0f) {
        StopState(i);
        return;
    }
    int8_t dir = rpm > 0.0f ? 1 : -1;
    if (s_motor[i].mode == MOTOR4_MODE_STOP) {
        s_rt[i].no_feedback_ticks = 0;
        s_rt[i].wrong_dir_ticks = 0;
    }
    if (s_motor[i].mode != MOTOR4_MODE_SPEED || dir != s_rt[i].command_dir) {
        s_rt[i].integral = 0.0f;
    }
    s_rt[i].command_dir = dir;
    s_motor[i].target_rpm = rpm;
    s_motor[i].mode = MOTOR4_MODE_SPEED;
}

HAL_StatusTypeDef Motor4_Init(void)
{
    uint32_t key = Lock();
    if (s_initialized) {
        StopAllInternal();
        Unlock(key);
        return HAL_BUSY;
    }

    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        s_motor[i] = (Motor4Telemetry){0};
        s_rt[i] = (Motor4Runtime){0};
        s_hw_dir[i] = 0;
    }

    bool config_ok = isfinite(MOTOR4_MAX_TARGET_RPM) &&
        MOTOR4_MAX_TARGET_RPM > 0.0f &&
        __HAL_TIM_GET_AUTORELOAD(&htim1) == 4199U;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (!isfinite(s_cfg[i].cpr) || s_cfg[i].cpr <= 0.0f ||
            !isfinite(s_cfg[i].kp) || s_cfg[i].kp < 0.0f ||
            !isfinite(s_cfg[i].ki) || s_cfg[i].ki < 0.0f ||
            (s_cfg[i].encoder_sign != 1 && s_cfg[i].encoder_sign != -1) ||
            (s_cfg[i].drive_sign != 1 && s_cfg[i].drive_sign != -1) ||
            __HAL_TIM_GET_AUTORELOAD(s_cfg[i].encoder) != 65535U) {
            config_ok = false;
        }
    }
    if (!config_ok) {
        LatchFault(MOTOR4_COUNT, MOTOR4_FAULT_CONFIG);
        Unlock(key);
        return HAL_ERROR;
    }

    StopAllInternal();
    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    s_watchdog_ms = HAL_GetTick();
    Unlock(key);

    /* HAL启动函数不放在关中断区中；TIM9此时尚未启动。
     * 无论哪个启动失败，共享STBY都会保持低电平。 */
    unsigned pwm_started = 0;
    unsigned enc_started = 0;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (HAL_TIM_PWM_Start(&htim1, s_cfg[i].channel) != HAL_OK) goto failed;
        ++pwm_started;
    }
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        __HAL_TIM_SET_COUNTER(s_cfg[i].encoder, 0U);
        s_rt[i].encoder_last = 0;
        if (HAL_TIM_Encoder_Start(s_cfg[i].encoder, TIM_CHANNEL_ALL) != HAL_OK) goto failed;
        ++enc_started;
    }

    key = Lock();
    s_watchdog_ms = HAL_GetTick();
    s_initialized = true;
    Unlock(key);
    return HAL_OK;

failed:
    key = Lock();
    LatchFault(MOTOR4_COUNT, MOTOR4_FAULT_INIT);
    Unlock(key);
    for (unsigned i = 0; i < enc_started; ++i) {
        (void)HAL_TIM_Encoder_Stop(s_cfg[i].encoder, TIM_CHANNEL_ALL);
    }
    for (unsigned i = 0; i < pwm_started; ++i) {
        (void)HAL_TIM_PWM_Stop(&htim1, s_cfg[i].channel);
    }
    return HAL_ERROR;
}

void Motor4_Stop(Motor4Id id)
{
    if (!ValidId(id)) return;
    uint32_t key = Lock();
    if (s_initialized) {
        StopState((unsigned)id);
        int16_t pwm[MOTOR4_COUNT];
        for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
            pwm[i] = i == (unsigned)id ? 0 : s_motor[i].pwm;
        }
        CommitOutputs(pwm);
    } else {
        HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    }
    Unlock(key);
}

void Motor4_StopAll(void)
{
    uint32_t key = Lock();
    if (s_initialized) StopAllInternal();
    else HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    Unlock(key);
}

void Motor4_FeedWatchdog(void)
{
    uint32_t key = Lock();
    if (s_initialized) s_watchdog_ms = HAL_GetTick();
    Unlock(key);
}

bool Motor4_SetOpenLoop(Motor4Id id, int16_t pwm)
{
    if (!ValidId(id)) return false;
    pwm = ClampPWM(pwm);
    uint32_t key = Lock();
    if (!s_initialized || (pwm != 0 && HasFault()) ||
        (pwm != 0 && !DirectionAllowed((unsigned)id, pwm > 0 ? 1 : -1))) {
        Unlock(key);
        return false;
    }

    s_watchdog_ms = HAL_GetTick();
    if (pwm == 0) {
        StopState((unsigned)id);
        int16_t outputs[MOTOR4_COUNT];
        for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
            outputs[i] = i == (unsigned)id ? 0 : s_motor[i].pwm;
        }
        CommitOutputs(outputs);
    } else {
        if (s_motor[id].mode == MOTOR4_MODE_STOP) {
            s_rt[id].no_feedback_ticks = 0;
            s_rt[id].wrong_dir_ticks = 0;
        }
        s_rt[id].integral = 0.0f;
        s_rt[id].command_dir = pwm > 0 ? 1 : -1;
        s_rt[id].open_pwm = pwm;
        s_motor[id].target_rpm = 0.0f;
        s_motor[id].mode = MOTOR4_MODE_OPEN;
    }
    Unlock(key);
    return true;
}

bool Motor4_SetSpeedRPM(Motor4Id id, float rpm)
{
    if (!ValidId(id) || !ValidSpeed(rpm)) return false;
    rpm = NormalizeSpeed(rpm);
    uint32_t key = Lock();
    if (!s_initialized || (rpm != 0.0f && HasFault()) ||
        (rpm != 0.0f && !DirectionAllowed((unsigned)id, rpm > 0.0f ? 1 : -1))) {
        Unlock(key);
        return false;
    }

    SetSpeedState((unsigned)id, rpm);
    s_watchdog_ms = HAL_GetTick();
    if (rpm == 0.0f) {
        int16_t pwm[MOTOR4_COUNT];
        for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
            pwm[i] = i == (unsigned)id ? 0 : s_motor[i].pwm;
        }
        CommitOutputs(pwm);
    }
    Unlock(key);
    return true;
}

bool Motor4_SetAllSpeedsRPM(float fl, float fr, float rl, float rr)
{
    float rpm[MOTOR4_COUNT] = { fl, fr, rl, rr };
    bool any = false;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (!ValidSpeed(rpm[i])) return false;
        rpm[i] = NormalizeSpeed(rpm[i]);
        if (rpm[i] != 0.0f) any = true;
    }

    uint32_t key = Lock();
    if (!s_initialized || (any && HasFault())) {
        Unlock(key);
        return false;
    }
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (rpm[i] != 0.0f &&
            !DirectionAllowed(i, rpm[i] > 0.0f ? 1 : -1)) {
            Unlock(key);
            return false;
        }
    }

    int16_t pwm[MOTOR4_COUNT];
    bool need_stop = false;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        SetSpeedState(i, rpm[i]);
        pwm[i] = rpm[i] == 0.0f ? 0 : s_motor[i].pwm;
        if (rpm[i] == 0.0f) need_stop = true;
    }
    s_watchdog_ms = HAL_GetTick();
    /* 非零新速度在下一个10ms tick生效；零目标立即撤销该路驱动。 */
    if (need_stop) CommitOutputs(pwm);
    Unlock(key);
    return true;
}

bool Motor4_ClearFaults(void)
{
    uint32_t key = Lock();
    bool allowed = s_initialized;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (s_motor[i].mode != MOTOR4_MODE_STOP || s_rt[i].quiet_ticks < QUIET_TICKS) {
            allowed = false;
        }
    }
    if (allowed) {
        for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
            s_motor[i].fault = MOTOR4_FAULT_NONE;
        }
        s_watchdog_ms = HAL_GetTick();
    }
    Unlock(key);
    return allowed;
}

bool Motor4_GetTelemetry(Motor4Id id, Motor4Telemetry *out)
{
    if (!ValidId(id) || out == NULL) return false;
    uint32_t key = Lock();
    *out = s_motor[id];
    Unlock(key);
    return true;
}

void Motor4_GetAllTelemetry(Motor4Telemetry out[MOTOR4_COUNT])
{
    if (out == NULL) return;
    uint32_t key = Lock();
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) out[i] = s_motor[i];
    Unlock(key);
}

void Motor4_ControlTick10ms(void)
{
    /* 计算和寄存器提交均为有界操作，无串口、等待或HAL_Delay。
     * 短暂屏蔽可屏蔽中断，避免更高优先级回调在计算中途改变命令。 */
    uint32_t key = Lock();
    if (!s_initialized) {
        Unlock(key);
        return;
    }
    int16_t pwm[MOTOR4_COUNT] = {0};

    /* 即使已锁存故障也继续读编码器，为停稳确认和故障清除提供依据。 */
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_cfg[i].encoder);
        uint16_t diff = (uint16_t)(now - s_rt[i].encoder_last);
        s_rt[i].encoder_last = now;
        int32_t delta = diff < 32768U ? (int32_t)diff : (int32_t)diff - 65536;
        delta *= s_cfg[i].encoder_sign * s_cfg[i].drive_sign;
        s_motor[i].delta = delta;
        s_motor[i].position += delta;
        s_motor[i].rpm_raw = (float)delta * 60.0f / (s_cfg[i].cpr * SAMPLE_S);
        s_motor[i].rpm += 0.25f * (s_motor[i].rpm_raw - s_motor[i].rpm);

        if (delta == 0) {
            if (s_rt[i].quiet_ticks < QUIET_TICKS) ++s_rt[i].quiet_ticks;
        } else {
            s_rt[i].quiet_ticks = 0;
        }
    }

    if (HasFault()) {
        Unlock(key);
        return;
    }
    if (AnyActive() &&
        (uint32_t)(HAL_GetTick() - s_watchdog_ms) >= MOTOR4_WATCHDOG_MS) {
        LatchFault(MOTOR4_COUNT, MOTOR4_FAULT_WATCHDOG);
        Unlock(key);
        return;
    }

    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (s_motor[i].mode == MOTOR4_MODE_STOP) continue;

        int32_t delta = s_motor[i].delta;
        if (delta == 0) {
            if (s_rt[i].no_feedback_ticks < NO_FEEDBACK_TICKS) {
                ++s_rt[i].no_feedback_ticks;
            }
        } else {
            s_rt[i].no_feedback_ticks = 0;
        }
        if (delta * s_rt[i].command_dir < 0) {
            if (s_rt[i].wrong_dir_ticks < WRONG_DIR_TICKS) ++s_rt[i].wrong_dir_ticks;
        } else {
            s_rt[i].wrong_dir_ticks = 0;
        }

        if (s_rt[i].no_feedback_ticks >= NO_FEEDBACK_TICKS) {
            LatchFault(i, MOTOR4_FAULT_NO_FEEDBACK);
            Unlock(key);
            return;
        }
        if (s_rt[i].wrong_dir_ticks >= WRONG_DIR_TICKS) {
            LatchFault(i, MOTOR4_FAULT_WRONG_DIRECTION);
            Unlock(key);
            return;
        }

        if (s_motor[i].mode == MOTOR4_MODE_OPEN) {
            pwm[i] = s_rt[i].open_pwm;
            continue;
        }

        float target = s_motor[i].target_rpm;
        int8_t dir = target > 0.0f ? 1 : -1;
        float magnitude = target > 0.0f ? target : -target;
        float error = magnitude - (float)dir * s_motor[i].rpm;
        float p = s_cfg[i].kp * error;
        float current = p + s_rt[i].integral;
        if ((current > 0.0f && current < MOTOR4_PWM_LIMIT) ||
            (current >= MOTOR4_PWM_LIMIT && error < 0.0f) ||
            (current <= 0.0f && error > 0.0f)) {
            s_rt[i].integral = Clamp(
                s_rt[i].integral + s_cfg[i].ki * SAMPLE_S * error,
                0.0f, (float)MOTOR4_PWM_LIMIT);
        }
        float u = Clamp(p + s_rt[i].integral, 0.0f, (float)MOTOR4_PWM_LIMIT);
        pwm[i] = (int16_t)((float)dir * u);
    }
    CommitOutputs(pwm);
    Unlock(key);
}
