#include "motor4.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>

/* 分工：主循环设置目标；TIM9中断每10ms统一采样并执行四路控制。
 * 四个编码器均设ARR=65535，即计数器在0..65535之间循环；
 * 为正确判断回绕后的方向，每个10ms周期的实际计数变化绝对值须小于32768。 */
/* 下列TICKS均以一次10ms中断为单位，不是毫秒数。 */
#define SAMPLE_S             0.01f
#define QUIET_TICKS          20U /* 连续200ms无计数，才认为已停稳 */
#define NO_FEEDBACK_TICKS    20U /* 运行时连续200ms无计数触发故障 */
#define WRONG_DIR_TICKS      10U /* 连续100ms计数反向触发故障 */
/* 四路共同的待机使能；拉低即可撤销全部驱动。 */
#define STBY_PORT            GPIOA
#define STBY_PIN             GPIO_PIN_4

#if MOTOR4_PWM_LIMIT < 1 || MOTOR4_PWM_LIMIT > 1000
#error "MOTOR4_PWM_LIMIT must be in 1..1000"
#endif

/* 不随运行改变的硬件/标定表：每轮在这里绑定编码器、PWM、方向脚和控制参数。
 * 运行状态放在Motor4Runtime和Motor4Telemetry中，避免把配置值与实时值混在一起。 */
typedef struct {
    TIM_HandleTypeDef *encoder; /* 该轮的编码器定时器 */
    uint32_t channel;           /* TIM1中负责该轮PWM的通道 */
    GPIO_TypeDef *in_port;      /* 两个方向引脚所在的GPIO端口 */
    uint16_t in1;
    uint16_t in2;
    float cpr;
    float kp;
    float ki;
    int8_t encoder_sign;
    int8_t drive_sign;
} Motor4Config;

/* 这张表把“轮子位置”关联到实际外设。逻辑轮序FL,FR,RL,RR对应接口A,D,B,C。
 * 表中channel属于TIM1，前面的htimX属于编码器，两者不是同一个定时器。
 * 接线汇总：
 *   A/FL：编码器TIM3 PA6/PA7，  PWM PA8(TIM1_CH1)，  方向PC0/PC1。
 *   D/FR：编码器TIM5 PA0/PA1，  PWM PA11(TIM1_CH4)，方向PC6/PC7。
 *   B/RL：编码器TIM2 PA15/PB3， PWM PA9(TIM1_CH2)， 方向PC2/PC3。
 *   C/RR：编码器TIM4 PB6/PB7，  PWM PA10(TIM1_CH3)，方向PC4/PC5。 */
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
    int16_t open_pwm;         /* 开环模式保存的PWM命令 */
    float integral;          /* PI积分项，单位为PWM千分比 */
    uint16_t encoder_last;   /* 上次采样时的原始硬件计数器值 */
    int8_t command_dir;      /* 最近非零命令方向：+1或-1；停止后仍保留 */
    uint8_t quiet_ticks;     /* 连续无计数次数，判断停稳和允许换向 */
    uint8_t no_feedback_ticks;
    uint8_t wrong_dir_ticks;
} Motor4Runtime;

/* 主循环和中断共享这些状态。volatile要求每次从内存读取；
 * 多字段读写的一致性仍需要下面的短临界区，不能只依赖volatile。 */
static volatile Motor4Telemetry s_motor[MOTOR4_COUNT];
static volatile Motor4Runtime s_rt[MOTOR4_COUNT];
static int8_t s_hw_dir[MOTOR4_COUNT];
static volatile bool s_initialized;
static volatile uint32_t s_watchdog_ms;

static uint32_t Lock(void)
{
    /* 保存原中断状态后屏蔽可屏蔽中断；Unlock恢复原状态，
     * 因此即使调用前已经关中断，也不会在退出时错误地打开中断。 */
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
    /* 先转无符号再比较，也能拒绝调用者传入的负轮号。 */
    return (unsigned)id < (unsigned)MOTOR4_COUNT;
}

static float Clamp(float x, float lo, float hi)
{
    /* 将数值夹在闭区间[lo,hi]；PI分别用它限制积分和最终输出。 */
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
    /* “活动”按运行模式判断，不按当前PWM判断；闭环暂时输出0也仍需刷新许可。 */
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (s_motor[i].mode != MOTOR4_MODE_STOP) return true;
    }
    return false;
}

/* 将四轮逻辑PWM一次提交到方向GPIO、TIM1的CCR和共享STBY。
 * 调用方位于短临界区，或来自TIM9中断。
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
    /* 计数器从0数到ARR，共ARR+1拍；当前ARR=4199，所以一周期4200拍。 */
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        pwm[i] = ClampPWM(logical[i]);
        /* 逻辑正转表示车辆前进；drive_sign把它转换为实际IN1/IN2极性。 */
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
            /* 其余电机运行时，本路PWM保持高且两方向脚均低，输出高阻滑行。
             * 因此日志pwm=0不能直接理解为该通道CCR=0。 */
            ccr = any ? period : 0U;
        } else {
            uint32_t magnitude = (uint32_t)(pwm[i] > 0 ? pwm[i] : -pwm[i]);
            /* PWM mode 1中CNT<CCR时为高：例如150得到630拍高电平。
             * 1000得到CCR=4200，大于ARR，整个周期持续高电平即100%。 */
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
    /* 停止只清目标和控制记忆，不清累计位置、测速结果或故障标志。
     * 刚停止时转子可能仍靠惯性旋转，所以要重新确认连续静止时间。 */
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
    /* source为某个轮号时只记录该轮；传入MOTOR4_COUNT则记录全部。
     * 使用|=保留已有故障。锁存后不会因为计数恢复就自行清除或重新启动。 */
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
    /* 首次启动或同向调速可接受；反向必须先STOP并连续200ms无计数。
     * command_dir在停止后保留，防止“停一下马上反转”绕过静止检查。 */
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
    /* 将非常接近0的浮点目标按停止处理，避免微小数值触发无反馈故障。 */
    return rpm > -0.01f && rpm < 0.01f ? 0.0f : rpm;
}

static void SetSpeedState(unsigned i, float rpm)
{
    /* 只更新这一轮的软件目标；调用者已经完成参数、故障和换向条件的校验。
     * 正常非零目标稍后由统一10ms控制周期采样并输出。 */
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
        /* 同向闭环变速保留积分，避免上层每次更新目标都从零重新调节。
         * 从开环/停止进入闭环或方向改变时才清积分。 */
        s_rt[i].integral = 0.0f;
    }
    s_rt[i].command_dir = dir;
    s_motor[i].target_rpm = rpm;
    s_motor[i].mode = MOTOR4_MODE_SPEED;
}

HAL_StatusTypeDef Motor4_Init(void)
{
    /* 初始化是上电流程的一部分；再次调用会明确停机并返回HAL_BUSY，
     * 不把反复初始化当成清除故障或重置位置的普通运行接口。 */
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

    /* 先校验计算依赖的关键参数和ARR；这不是对所有CubeMX配置的全量校验。
     * 编码器计数范围或PWM周期不符时，不继续带着错误参数启动控制。 */
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
    /* 部分外设已经启动也要回收；先让STBY为低，再逐个停止已启动通道。 */
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
    /* 只把指定轮子的目标与PWM置零，其余三轮保留原输出。
     * 输出提交函数会根据是否还有其他轮子运行，决定共享STBY状态。 */
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
    /* 未初始化也能拉低STBY；因此上电早期出错路径可以用此接口撤销驱动。 */
    uint32_t key = Lock();
    if (s_initialized) StopAllInternal();
    else HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    Unlock(key);
}

void Motor4_FeedWatchdog(void)
{
    /* 记录主循环最近确认“仍允许运行”的毫秒时刻；不改变目标转速或PWM。 */
    uint32_t key = Lock();
    if (s_initialized) s_watchdog_ms = HAL_GetTick();
    Unlock(key);
}

bool Motor4_SetOpenLoop(Motor4Id id, int16_t pwm)
{
    /* 开环用于接线/方向测试：只固定占空比，不追踪某个RPM。
     * 非零命令保存下来，由下一个10ms控制周期统一输出。 */
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
    /* 请求可在主循环提交。拒绝时保持原目标，调用者需检查返回值。
     * 零目标是停机请求，不等待下一次PI；有效非零目标由下一次PI计算输出。 */
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
    /* 两阶段处理：先检查四个目标和换向条件，全部通过后才修改状态。
     * 配合临界区，使四轮不可能只更新两轮就被控制中断取走半份命令。 */
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
    /* 清故障与重新使能分开：必须四路均STOP且连续静止，清除后仍保持停止。
     * 遥控/路线应用不会在这里自动恢复故障前的目标。 */
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
    /* 返回的是调用时刻的一份副本，out不是内部状态指针；调用者修改它不影响控制。
     * 临界区同时避免64位position在Cortex-M4上被分两次读取时夹入中断。 */
    if (!ValidId(id) || out == NULL) return false;
    uint32_t key = Lock();
    *out = s_motor[id];
    Unlock(key);
    return true;
}

void Motor4_GetAllTelemetry(Motor4Telemetry out[MOTOR4_COUNT])
{
    /* 位置是64位，多轮还要用于共同估算转角；一次性复制可避免读到
     * 中断更新前后的混合数据，返回后主循环便可慢慢格式化串口日志。 */
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
        /* 先按16位无符号数做差，再转换为最短的有符号变化。
         * 例如65534 -> 2为+4，2 -> 65534为-4，跨过零也不会误判为巨跳。 */
        uint16_t diff = (uint16_t)(now - s_rt[i].encoder_last);
        s_rt[i].encoder_last = now;
        int32_t delta = diff < 32768U ? (int32_t)diff : (int32_t)diff - 65536;
        /* encoder_sign校准AB相序，drive_sign校准轮子安装朝向。
         * 输出和反馈使用相同的安装修正，确保正目标对应正反馈。 */
        delta *= s_cfg[i].encoder_sign * s_cfg[i].drive_sign;
        s_motor[i].delta = delta;
        s_motor[i].position += delta;
        /* delta/CPR是10ms内转过的圈数，除0.01换成圈/秒，再乘60得到RPM。
         * EMA滤波每次向原始值靠近25%，减小脉冲离散造成的跳动，也会引入延迟。 */
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
        /* 无符号时间差可跨越HAL毫秒计数器回绕；超时只在有运动命令时触发。 */
        LatchFault(MOTOR4_COUNT, MOTOR4_FAULT_WATCHDOG);
        Unlock(key);
        return;
    }

    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (s_motor[i].mode == MOTOR4_MODE_STOP) continue;

        int32_t delta = s_motor[i].delta;
        /* 两项反馈检查同时用于开环和闭环。它们只检查计数现象，
         * 无反馈可能来自停转或接线故障，不能据此测出堵转电流。 */
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
        /* 先把正反转统一为“沿命令方向的速度大小”来调节。
         * 误差>0表示转得太慢；误差<0则降低占空比，不在此处反向制动。 */
        float error = magnitude - (float)dir * s_motor[i].rpm;
        float p = s_cfg[i].kp * error;
        float current = p + s_rt[i].integral;
        /* 每10ms积分增加KI*0.01*error，0.01是以秒表示的采样间隔。
         * 抗积分饱和：输出在范围内才正常积分；达到上/下限时，
         * 只允许能使输出回到范围内的误差继续积分，防止积分越积越大。 */
        if ((current > 0.0f && current < MOTOR4_PWM_LIMIT) ||
            (current >= MOTOR4_PWM_LIMIT && error < 0.0f) ||
            (current <= 0.0f && error > 0.0f)) {
            s_rt[i].integral = Clamp(
                s_rt[i].integral + s_cfg[i].ki * SAMPLE_S * error,
                0.0f, (float)MOTOR4_PWM_LIMIT);
        }
        float u = Clamp(p + s_rt[i].integral, 0.0f, (float)MOTOR4_PWM_LIMIT);
        /* 最后加回命令方向。PI只决定占空比大小，硬件极性由CommitOutputs转换。 */
        pwm[i] = (int16_t)((float)dir * u);
    }
    CommitOutputs(pwm);
    Unlock(key);
}
