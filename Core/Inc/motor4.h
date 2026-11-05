#ifndef MOTOR4_H
#define MOTOR4_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* 四路电机底层：上层给出每轮目标转速，本模块负责读编码器、调节PWM。
 * TIM1四通道输出20kHz PWM；TIM2/3/4/5分别读四个AB编码器；
 * TIM9每10ms调用一次控制函数。麦轮运动分解和路线时序由上层完成。
 * PWM是驱动信号的占空比，RPM是输出轴每分钟转数，两者不是同一个量。
 * 本模块只有反馈/时序故障检测，没有电机电流采样或硬件限流。 */
/* PWM千分比上限：1000=100%，150=15%；负号由方向引脚处理。 */
#define MOTOR4_PWM_LIMIT         1000
/* 允许接收的转速上限，不表示负载下电机一定能达到该转速。 */
#define MOTOR4_MAX_TARGET_RPM    110.0f
/* PI参数：P根据当前误差立即修正；I累积误差以补偿长期偏差。
 * 此处输出单位是PWM千分比，误差单位是RPM，积分按秒计算。 */
#define MOTOR4_KP                10.0f
#define MOTOR4_KI                10.0f
/* 主循环超过此时间没有确认仍可运行，10ms控制中断便会撤销驱动。 */
#define MOTOR4_WATCHDOG_MS       500U

/* CPR：减速箱输出轴转一圈时，定时器实际累计多少个计数。
 * 编码器模式同时利用A/B两相的边沿，通常称四倍频；它不同于商品页
 * 所写的电机轴单相脉冲数，还包含减速比。第一台已初步标定3960，
 * 其他同型号电机暂用相同值。CPR会直接影响测速、里程和估算转角。
 * #ifndef允许编译时单独覆盖某个轮子的参数，不必改动模块内部。 */
#ifndef MOTOR4_FL_CPR
#define MOTOR4_FL_CPR 3960.0f
#endif
#ifndef MOTOR4_FR_CPR
#define MOTOR4_FR_CPR 3960.0f
#endif
#ifndef MOTOR4_RL_CPR
#define MOTOR4_RL_CPR 3960.0f
#endif
#ifndef MOTOR4_RR_CPR
#define MOTOR4_RR_CPR 3960.0f
#endif

/* 物理接口：A=FL左前，B=RL左后，C=RR右后，D=FR右前。
 * ENCODER_SIGN沿用各物理通道已经验证的相序：A/C=-1，B/D=+1。
 * ENCODER_SIGN校准相序：驱动极性为+1时，正PWM得到正计数。
 * DRIVE_SIGN校准安装方向：设为-1会同时反转物理驱动和反馈，使逻辑正
 * 转始终表示该轮推动小车向前。当前安装校准：前轮=-1，后轮=+1。
 * 不要只反转驱动而不反转反馈；本模块已将DRIVE_SIGN同时用于两者。 */
#ifndef MOTOR4_FL_ENCODER_SIGN
#define MOTOR4_FL_ENCODER_SIGN (-1)
#endif
#ifndef MOTOR4_FR_ENCODER_SIGN
#define MOTOR4_FR_ENCODER_SIGN (+1)
#endif
#ifndef MOTOR4_RL_ENCODER_SIGN
#define MOTOR4_RL_ENCODER_SIGN (+1)
#endif
#ifndef MOTOR4_RR_ENCODER_SIGN
#define MOTOR4_RR_ENCODER_SIGN (-1)
#endif
#ifndef MOTOR4_FL_DRIVE_SIGN
#define MOTOR4_FL_DRIVE_SIGN (-1)
#endif
#ifndef MOTOR4_FR_DRIVE_SIGN
#define MOTOR4_FR_DRIVE_SIGN (-1)
#endif
#ifndef MOTOR4_RL_DRIVE_SIGN
#define MOTOR4_RL_DRIVE_SIGN (+1)
#endif
#ifndef MOTOR4_RR_DRIVE_SIGN
#define MOTOR4_RR_DRIVE_SIGN (+1)
#endif

/* API和数组始终使用FL、FR、RL、RR顺序；注意它不是驱动板ABCD顺序。
 * FL/FR/RL/RR分别是Front Left/Front Right/Rear Left/Rear Right。 */
typedef enum {
    MOTOR4_FL = 0,   /* D24A A：左前 */
    MOTOR4_FR,       /* D24A D：右前 */
    MOTOR4_RL,       /* D24A B：左后 */
    MOTOR4_RR,       /* D24A C：右后 */
    MOTOR4_COUNT
} Motor4Id;

typedef enum {
    MOTOR4_MODE_STOP = 0, /* 撤销驱动，编码器仍继续采样 */
    MOTOR4_MODE_OPEN,     /* 开环：固定PWM，转速会随电池和负载变化 */
    MOTOR4_MODE_SPEED     /* 闭环：用编码器反馈和PI调节PWM，追踪目标RPM */
} Motor4Mode;

/* 故障是位标志，可以用按位或组合：例如fault=2是方向不符。
 * 反馈故障记在发现问题的轮子上，但任一路故障都会停止全部四轮。 */
typedef enum {
    MOTOR4_FAULT_NONE            = 0U,
    MOTOR4_FAULT_NO_FEEDBACK     = 1U << 0, /* 1：命令运行但连续200ms无计数 */
    MOTOR4_FAULT_WRONG_DIRECTION = 1U << 1, /* 2：连续100ms计数方向与命令相反 */
    MOTOR4_FAULT_WATCHDOG        = 1U << 2, /* 4：运行许可刷新超时 */
    MOTOR4_FAULT_CONFIG          = 1U << 3, /* 8：关键参数或定时器周期不符 */
    MOTOR4_FAULT_INIT            = 1U << 4  /* 16：PWM或编码器启动失败 */
} Motor4Fault;

/* 主循环通过GetTelemetry读取这些快照，避免直接访问中断中的状态。
 * 方向均为校准后的逻辑方向：正值推动该轮向小车前方行驶。 */
typedef struct {
    float rpm_raw;    /* 最近10ms计数换算的转速，低速时会有量化跳动 */
    float rpm;        /* 平滑后的转速，PI控制和日志使用它 */
    float target_rpm; /* 闭环目标；开环模式下为0，不表示电机没有转动 */
    int64_t position; /* 从初始化起累计的有符号计数；停止不会清零 */
    int32_t delta;    /* 最近一个10ms周期新增的有符号计数 */
    int16_t pwm;       /* 逻辑PWM千分比；0代表本路停止，不代表原始CCR为0 */
    uint8_t mode;
    uint8_t fault;     /* 故障位锁存；任一路故障均使四路停止 */
} Motor4Telemetry;

/* 数据关系示例：CPR=3960时，position=3960表示相对初始化位置正向累计一圈；
 * rpm=15表示当前约15转/分钟。位置可以继续累加，转速则可在停机后降为0。
 * position并不是米，rpm也不是车身m/s；车身换算还需要轮径和底盘几何关系。 */

/* 先执行Cube生成的GPIO/TIM1/2/3/4/5初始化，再调用一次本函数。
 * 本模块启动PWM和编码器；TIM9的10ms中断由main最后启动。
 * 初始化成功后仍处于停止状态，不会自行转动。 */
HAL_StatusTypeDef Motor4_Init(void);
/* 只由10ms定时中断调用：采样 -> 测速 -> 检查故障 -> PI -> 提交输出。
 * 主循环不要再调用它，否则采样间隔和测速计算会不一致。 */
void Motor4_ControlTick10ms(void);

/* 接受有效运动命令会更新软件看门狗；重复同向速度命令不清积分。
 * 超出最大目标、NaN/Inf、故障锁存或未满足换向条件会返回false，
 * 原运动命令保持原状，调用方应检查返回值并视需要StopAll。
 * 开环PWM夹到±MOTOR4_PWM_LIMIT；速度命令超出±MOTOR4_MAX_TARGET_RPM会拒绝。
 * PWM为千分比，1000对应100%占空比；本模块没有电流采样或限流。 */
bool Motor4_SetOpenLoop(Motor4Id id, int16_t pwm);
/* 单轮闭环主要供独立测试；整车运动优先由mecanum调用下面的四轮一起设速接口。 */
bool Motor4_SetSpeedRPM(Motor4Id id, float rpm);

/* 参数依次为左前、右前、左后、右后，单位RPM，允许正负或0。
 * 四路命令先全部校验，再一起生效，不会只改变其中一部分。
 * 同向变速可直接更新；换向须先停止并连续200ms无计数。 */
bool Motor4_SetAllSpeedsRPM(float fl, float fr, float rl, float rr);

/* 立即撤销驱动，保留位置和故障信息。
 * 单路停止时不会关闭其他电机；所有路停止时共享STBY拉低。
 * 重复停止不会重置静止计时。Stop函数本身不喂看门狗。 */
void Motor4_Stop(Motor4Id id);
void Motor4_StopAll(void);

/* 这是软件运行许可，不是STM32独立看门狗，也不会复位芯片。
 * 有运动时主循环必须持续确认运行条件，并在500ms超时前喂一次。
 * 超时达到500ms后，在下一个控制tick全停并锁存故障。
 * CPU调试暂停或中断不能执行时，软件看门狗也不能保证停机。 */
void Motor4_FeedWatchdog(void);

/* 四路均STOP且连续200ms无编码器变化才可清故障，不会自动启动。 */
bool Motor4_ClearFaults(void);
/* 单路读取时校验id和out；成功返回true。四路读取获得同一临界区快照，
 * 转角计算应优先使用四路快照，避免中断恰好夹在几次单路读取之间。 */
bool Motor4_GetTelemetry(Motor4Id id, Motor4Telemetry *out);
void Motor4_GetAllTelemetry(Motor4Telemetry out[MOTOR4_COUNT]);

#endif
