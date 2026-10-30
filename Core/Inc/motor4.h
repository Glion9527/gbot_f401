#ifndef MOTOR4_H
#define MOTOR4_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* 手动导入时将本文件命名为 Core/Inc/motor4.h。
 * 10ms控制周期、TIM1四通道20kHz PWM、四路AB编码器。
 * 这些软件退出措施不是电机电流检测或硬件限流。 */
#define MOTOR4_PWM_LIMIT         1000
#define MOTOR4_MAX_TARGET_RPM    110.0f
#define MOTOR4_KP                10.0f
#define MOTOR4_KI                10.0f
#define MOTOR4_WATCHDOG_MS       500U

/* CPR：输出轴每转的四倍频计数。第一台已初步标定3960，其他同型号
 * 电机先用相同初值，逐路验证编码器和方向后再四轮联动。 */
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

typedef enum {
    MOTOR4_FL = 0,   /* D24A A：左前 */
    MOTOR4_FR,       /* D24A D：右前 */
    MOTOR4_RL,       /* D24A B：左后 */
    MOTOR4_RR,       /* D24A C：右后 */
    MOTOR4_COUNT
} Motor4Id;

typedef enum {
    MOTOR4_MODE_STOP = 0,
    MOTOR4_MODE_OPEN,
    MOTOR4_MODE_SPEED
} Motor4Mode;

typedef enum {
    MOTOR4_FAULT_NONE            = 0U,
    MOTOR4_FAULT_NO_FEEDBACK     = 1U << 0,
    MOTOR4_FAULT_WRONG_DIRECTION = 1U << 1,
    MOTOR4_FAULT_WATCHDOG        = 1U << 2,
    MOTOR4_FAULT_CONFIG          = 1U << 3,
    MOTOR4_FAULT_INIT            = 1U << 4
} Motor4Fault;

typedef struct {
    float rpm_raw;
    float rpm;
    float target_rpm;
    int64_t position;
    int32_t delta;
    int16_t pwm;       /* 逻辑PWM千分比；0代表本路停止，不代表原始CCR为0 */
    uint8_t mode;
    uint8_t fault;     /* 故障位锁存；任一路故障均使四路停止 */
} Motor4Telemetry;

/* 先执行Cube生成的GPIO/TIM1/2/3/4/5初始化，再调用一次本函数。
 * 本模块启动PWM和编码器；TIM9的10ms中断由main最后启动。 */
HAL_StatusTypeDef Motor4_Init(void);
void Motor4_ControlTick10ms(void);

/* 接受有效运动命令会更新软件看门狗；重复同向速度命令不清积分。
 * 超出最大目标、NaN/Inf、故障锁存或未满足换向条件会返回false，
 * 原运动命令保持原状，调用方应检查返回值并视需要StopAll。
 * 开环PWM夹到±MOTOR4_PWM_LIMIT；速度命令超过110RPM会拒绝。
 * PWM为千分比，1000对应100%占空比；本模块没有电流采样或限流。 */
bool Motor4_SetOpenLoop(Motor4Id id, int16_t pwm);
bool Motor4_SetSpeedRPM(Motor4Id id, float rpm);

/* 四路命令先全部校验，再一起生效，不会只改变其中一部分。 */
bool Motor4_SetAllSpeedsRPM(float fl, float fr, float rl, float rr);

/* 立即撤销驱动，保留位置和故障信息。
 * 单路停止时不会关闭其他电机；所有路停止时共享STBY拉低。
 * 重复停止不会重置静止计时。Stop函数本身不喂看门狗。 */
void Motor4_Stop(Motor4Id id);
void Motor4_StopAll(void);

/* 有运动时主循环必须持续确认运行条件，再至少每500ms喂一次。
 * 停止调用/主循环阻塞会在下一个控制tick全停并锁存故障。
 * CPU调试暂停或中断不能执行时，软件看门狗也不能保证停机。 */
void Motor4_FeedWatchdog(void);

/* 四路均STOP且连续200ms无编码器变化才可清故障，不会自动启动。 */
bool Motor4_ClearFaults(void);
bool Motor4_GetTelemetry(Motor4Id id, Motor4Telemetry *out);
void Motor4_GetAllTelemetry(Motor4Telemetry out[MOTOR4_COUNT]);

#endif
