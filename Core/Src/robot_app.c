#include "robot_app.h"
#include "motor4.h"
#include "mecanum.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 应用层运行在 main 的主循环中：读按钮与轮速 → 检查停止条件 → 更新路线 → 打印。
 * 电机层的 10ms 中断独立进行测速和调速；这里改变的是目标，不直接计算 PWM。
 * #if 会在编译时只保留选中模式的代码，串口文本也对应这个模式。 */
#if ROBOT_APP_MODE < 0 || ROBOT_APP_MODE > 7
#error "ROBOT_APP_MODE must be 0..7"
#endif

#if ROBOT_APP_MODE == 7
#include "remote_control.h"

/* 遥控模式不进入原按键路线：上电停止，等待串口握手和显式使能。 */
void RobotApp_Init(void)
{
    if (RemoteControl_Init() != HAL_OK) Error_Handler();
}

void RobotApp_Poll(void)
{
    /* 模式7不执行下面的本地按键状态机或普通文本日志；通信和控制许可统一
     * 由remote_control维护，串口协议遥测也由它发送。 */
    RemoteControl_Poll();
}

#else /* 保留原有0..6测试/路线模式 */

/* 状态机：每次 Poll 只检查当前阶段，等待时不占住主循环。 */
enum {
    WAIT_RELEASE,          /* 上电/停机后，先等 USER 松开 */
    RELEASE_FILTER,        /* 确认松开稳定，并等四轮静止 */
    WAIT_PRESS,            /* 已就绪，等待下一次按下 */
    PRESS_FILTER,          /* 确认按下稳定，排除瞬时抖动 */
    WAIT_CLICK_RELEASE,    /* 点击模式：按下已确认，等待松开 */
    CLICK_RELEASE_FILTER,  /* 点击模式：确认这次松开稳定 */
    START_DELAY,           /* 完整点击后延迟 1s，此时电机仍停止 */
    RUNNING                /* 运行中，处理按键停止、总超时和路线 */
};

/* s_ 前缀表示本文件内部保存的状态；static 使其他 .c 文件不能直接访问它们。 */
static uint8_t s_state;
#if ROBOT_APP_MODE == 0
static unsigned s_next_wheel;
static unsigned s_active_wheel;
#else
static uint32_t s_delay_ms;
#endif
static uint32_t s_key_ms;          /* 最近一次按键电平变化的消抖计时起点 */
static uint32_t s_start_ms;        /* 本次运动命令首次被接受的时刻 */
static uint32_t s_last_motion_ms;  /* 最近发现任一轮编码器变化的时刻 */
static uint32_t s_print_ms;        /* 上一组串口状态的发送时刻 */
static int64_t s_last_position[MOTOR4_COUNT]; /* 与本次位置比较，判断是否还在滑行 */
#if ROBOT_APP_MODE == 1
typedef enum {
    ROUTE_STRAIGHT_1,   /* 第一段直行，含起步加速；四轮同目标，没有车头角度反馈 */
    ROUTE_ARC_LEFT,     /* 保持向前，同时由编码器角度闭环控制左转 */
    ROUTE_STRAIGHT_2    /* 第二段直行，仍用同一角度反馈微调朝向 */
} RoutePhase;
static RoutePhase s_route_phase;
static uint32_t s_route_update_ms;
static uint32_t s_arc_settle_ms;
static bool s_arc_settling;
static bool s_ramp_complete;
static int64_t s_arc_start_position[MOTOR4_COUNT]; /* 弯道开始时的四轮位置，作为角度零点 */
static double s_route_yaw_deg;     /* 自弯道开始累计的估计转角，单位度 */
static float s_turn_rpm;           /* 上次下发的转向分量，用于平滑改变曲率 */
static bool s_route_command_valid;
static float s_route_last_forward;
static float s_route_last_turn;

/* 每路使用各自的输出轴四倍频CPR；顺序必须是FL,FR,RL,RR。 */
static const double s_route_cpr[MOTOR4_COUNT] = {
    MOTOR4_FL_CPR, MOTOR4_FR_CPR, MOTOR4_RL_CPR, MOTOR4_RR_CPR
};
#endif

/* 按逻辑轮序FL,FR,RL,RR索引；物理通道对应A,D,B,C。 */
static const char *const s_names[MOTOR4_COUNT] = {
    "A/FL", "D/FR", "B/RL", "C/RR"
};

/* 按物理接口A、B、C、D依次测试和打印。 */
static const Motor4Id s_test_order[MOTOR4_COUNT] = {
    MOTOR4_FL, MOTOR4_RL, MOTOR4_RR, MOTOR4_FR
};

#if ROBOT_APP_MODE == 1
#define MOVEMENT_NAME "forward"
#elif ROBOT_APP_MODE == 2
#define MOVEMENT_NAME "backward"
#elif ROBOT_APP_MODE == 3
#define MOVEMENT_NAME "left"
#elif ROBOT_APP_MODE == 4
#define MOVEMENT_NAME "right"
#elif ROBOT_APP_MODE == 5
#define MOVEMENT_NAME "ccw"
#elif ROBOT_APP_MODE == 6
#define MOVEMENT_NAME "cw"
#endif

/* 通过 USART2 发送调试文本；发送是同步的，完成后主循环才继续。
 * 50ms 是单次发送超时上限，日志周期与电机 10ms 控制周期互相独立。 */
static void Send(const char *text)
{
    size_t length = strlen(text);
    if (length > 0U && length <= 65535U) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)text,
                              (uint16_t)length, 50U);
    }
}

/* 任何一路有锁存故障，整车都不再接受本次运行。应用层不会自动清故障。 */
static bool HasFault(const Motor4Telemetry t[MOTOR4_COUNT])
{
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (t[i].fault != 0U) return true;
    }
    return false;
}

/* 四轮编码器均连续至少 250ms 没有被观察到变化，才允许准备下一次启动。
 * uint32_t 的“当前值 - 起点”写法也能跨过 HAL_GetTick 的计数回绕。 */
static bool WheelsStill(uint32_t now)
{
    return (uint32_t)(now - s_last_motion_ms) >= 250U;
}

#if ROBOT_APP_MODE == 1
/* 每次开始/结束都清除路线历史，避免下一次点击沿用上次的角度或转向量。 */
static void ResetRoute(void)
{
    s_route_phase = ROUTE_STRAIGHT_1;
    s_route_update_ms = 0U;
    s_arc_settle_ms = 0U;
    s_arc_settling = false;
    s_ramp_complete = true;
    s_route_yaw_deg = 0.0;
    s_turn_rpm = 0.0f;
    s_route_command_valid = false;
    s_route_last_forward = 0.0f;
    s_route_last_turn = 0.0f;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) s_arc_start_position[i] = 0;
}

/* 这里的 forward/turn 都是轮速 RPM 分量；left=0 表示不主动横向平移。
 * mecanum 会把它们换成 FL/FR/RL/RR 的四个目标，再交给电机层。 */
static bool SetRouteCommand(float forward, float turn)
{
    /* 相同目标不重复提交；同向目标更新由电机层保留PI和反馈保护计时。 */
    if (s_route_command_valid && forward == s_route_last_forward &&
        turn == s_route_last_turn) return true;
    if (!Mecanum_SetMixRPM(forward, 0.0f, turn)) return false;
    s_route_last_forward = forward;
    s_route_last_turn = turn;
    s_route_command_valid = true;
    return true;
}

static const char *RoutePhaseName(void)
{
    /* 英文短名用于串口诊断，状态机判断仍使用上面的枚举值。 */
    if (s_route_phase == ROUTE_ARC_LEFT) return "arc_left";
    if (s_route_phase == ROUTE_STRAIGHT_2) return "straight_2";
    return "straight_1";
}

static long AngleX10(double angle)
{
    /* 串口使用整数的十倍值：yaw_x10=900 表示 90.0°，便于不启用浮点 printf。 */
    double scaled = angle * 10.0;
    /* 日志也避免异常累计计数造成浮点转32位整数越界。 */
    if (scaled > (double)INT32_MAX) return (long)INT32_MAX;
    if (scaled < (double)INT32_MIN) return (long)INT32_MIN;
    return (long)scaled;
}

static long RouteYawX10(void)
{
    return AngleX10(s_route_yaw_deg);
}

static void PrintSegment(void)
{
    char line[112];
    int n = snprintf(line, sizeof(line), "SEGMENT phase=%s yaw_x10=%ld\r\n",
                     RoutePhaseName(), RouteYawX10());
    if (n > 0 && (size_t)n < sizeof(line)) Send(line);
}

static void UpdateRouteYaw(const Motor4Telemetry t[MOTOR4_COUNT])
{
    double revolutions[MOTOR4_COUNT];
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        /* 先转double再相减，不对int64累计位置做可能溢出的整数减法。 */
        revolutions[i] = ((double)t[i].position -
                          (double)s_arc_start_position[i]) / s_route_cpr[i];
    }
    /* 每轮相对弯道起点的计数变化 / CPR = 从弯道开始转过的圈数（带正负号）。
     * 标准 X 型麦轮：左侧轮少走、右侧轮多走产生正转角，因此组合为
     * -FL + FR - RL + RR。将轮周长 πD 和角度换算合并后得到：
     * yaw(度) = 90D/(L+W) × (-n_FL+n_FR-n_RL+n_RR)。
     * D/L/W 只需使用相同长度单位。此估计不含第一段直行的车头偏差，
     * 也不能直接检测地面打滑；有效轮径、CPR 和安装误差需落地核对。 */
    s_route_yaw_deg = 90.0 * (double)ROBOT_APP_WHEEL_DIAMETER_MM /
        ((double)ROBOT_APP_WHEELBASE_MM + (double)ROBOT_APP_TRACK_MM) *
        (-revolutions[MOTOR4_FL] + revolutions[MOTOR4_FR] -
          revolutions[MOTOR4_RL] + revolutions[MOTOR4_RR]);
}
#endif

/* 只提交本次动作的第一条目标；返回 false 时由上层统一停止。
 * “命令被接受”不等于电机立刻达到目标，实际追踪过程由 motor4 的 PI 完成。 */
static bool StartCommand(void)
{
#if ROBOT_APP_MODE == 0
    /* s_next_wheel是A/B/C/D测试序号，经s_test_order换成电机层的逻辑轮号。
     * 只有命令接受成功才推进下一轮，避免被拒绝后悄悄跳过接线测试。 */
    s_active_wheel = (unsigned)s_test_order[s_next_wheel];
    if (!Motor4_SetOpenLoop((Motor4Id)s_active_wheel, ROBOT_APP_TEST_PWM)) {
        return false;
    }
    s_next_wheel = (s_next_wheel + 1U) % MOTOR4_COUNT;
    return true;
#elif ROBOT_APP_MODE == 1
    ResetRoute();
    return SetRouteCommand(ROBOT_APP_START_RPM, 0.0f);
#elif ROBOT_APP_MODE == 2
    return Mecanum_SetMixRPM(-ROBOT_APP_TEST_RPM, 0.0f, 0.0f);
#elif ROBOT_APP_MODE == 3
    return Mecanum_SetMixRPM(0.0f, ROBOT_APP_TEST_RPM, 0.0f);
#elif ROBOT_APP_MODE == 4
    return Mecanum_SetMixRPM(0.0f, -ROBOT_APP_TEST_RPM, 0.0f);
#elif ROBOT_APP_MODE == 5
    return Mecanum_SetMixRPM(0.0f, 0.0f, ROBOT_APP_TEST_RPM);
#else
    return Mecanum_SetMixRPM(0.0f, 0.0f, -ROBOT_APP_TEST_RPM);
#endif
}

#if ROBOT_APP_MODE == 1
static bool UpdateRoute(uint32_t now, const Motor4Telemetry t[MOTOR4_COUNT])
{
    /* 路线是车身动作的外层控制，电机层仍独立每10ms调节各轮速度。
     * now和s_start_ms都来自HAL毫秒时钟；Poll快慢只影响本次是否需要更新目标。 */
    uint32_t elapsed = (uint32_t)(now - s_start_ms);
    if (s_route_phase == ROUTE_STRAIGHT_1) {
        if (elapsed >= ROBOT_APP_FIRST_STRAIGHT_MS) {
            /* 总运动时间到 5s 后开始弯道，不停下车辆。
             * t 来自 GetAllTelemetry 的一次原子快照，四路基准同时保存；
             * 原子快照指复制期间不会被电机中断改到一半，四路数据彼此一致。 */
            for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
                s_arc_start_position[i] = t[i].position;
            }
            s_route_phase = ROUTE_ARC_LEFT;
            s_route_yaw_deg = 0.0;
            s_turn_rpm = 0.0f;
            s_arc_settling = false;
            s_ramp_complete = true;
            s_route_update_ms = now;
            if (!SetRouteCommand(ROBOT_APP_TEST_RPM, 0.0f)) return false;
            PrintSegment();
            return true;
        }
        if (s_ramp_complete) return true;
        if (elapsed >= ROBOT_APP_RAMP_MS) {
            if (!SetRouteCommand(ROBOT_APP_TEST_RPM, 0.0f)) return false;
            s_ramp_complete = true;
            return true;
        }
        /* 起步每隔至少 10ms 更新目标，按实际经过时间插值：15 → 85RPM。
         * 这里只限制目标变化速度，并不是每次等待 10ms。 */
        if ((uint32_t)(now - s_route_update_ms) < 10U) return true;
        float target = ROBOT_APP_START_RPM +
            (ROBOT_APP_TEST_RPM - ROBOT_APP_START_RPM) *
            ((float)elapsed / (float)ROBOT_APP_RAMP_MS);
        if (!SetRouteCommand(target, 0.0f)) return false;
        s_route_update_ms = now;
        return true;
    }

    UpdateRouteYaw(t);
    double error = (double)ROBOT_APP_TURN_DEG - s_route_yaw_deg;
    bool segment_changed = false;
    if (s_route_phase == ROUTE_ARC_LEFT) {
        /* 瞬间碰到 90° 不立即结束弯道：需在 ±1° 内连续保持 150ms。
         * 期间若离开误差区间就重新计时，减小短暂波动造成的误判。 */
        if (error >= -(double)ROBOT_APP_ARC_TOLERANCE_DEG &&
            error <= (double)ROBOT_APP_ARC_TOLERANCE_DEG) {
            if (!s_arc_settling) {
                s_arc_settle_ms = now;
                s_arc_settling = true;
            } else if ((uint32_t)(now - s_arc_settle_ms) >= ROBOT_APP_ARC_SETTLE_MS) {
                s_route_phase = ROUTE_STRAIGHT_2;
                segment_changed = true;
            }
        } else s_arc_settling = false;
    }

    uint32_t step_ms = (uint32_t)(now - s_route_update_ms);
    if (step_ms >= 10U) {
        /* 角度外环：误差大时保持最大转向量，剩余不足 20° 时按比例收小。
         * 超过 90° 后误差为负，可产生小幅右转修正；电机层另外有轮速 PI。
         * 这里的 27.5RPM 是混合输入的轮速分量，不能当成车头转速。 */
        double requested = (double)ROBOT_APP_ARC_TURN_RPM * error /
                           (double)ROBOT_APP_ARC_SLOWDOWN_DEG;
        if (requested > (double)ROBOT_APP_ARC_TURN_RPM) requested = ROBOT_APP_ARC_TURN_RPM;
        if (requested < -(double)ROBOT_APP_ARC_TURN_RPM) requested = -ROBOT_APP_ARC_TURN_RPM;
        float turn = (float)requested;
        /* 按真实经过时间限制旋转分量变化，0到最大值至少300ms。 */
        float max_step = ROBOT_APP_ARC_TURN_RPM *
                         ((float)step_ms / (float)ROBOT_APP_ARC_RAMP_MS);
        if (turn > s_turn_rpm + max_step) turn = s_turn_rpm + max_step;
        if (turn < s_turn_rpm - max_step) turn = s_turn_rpm - max_step;
        float forward = ROBOT_APP_TEST_RPM - fabsf(turn);
        /* 对 left=0 的混合，左轮目标=forward-turn，右轮=forward+turn。
         * 当前配置最大左转时：forward=57.5、turn=27.5，故左轮=30RPM、
         * 右轮=85RPM；四轮始终向前，以左右轮速度差走弧线。
         * 外轮最多 85RPM，内轮至少 30RPM，超调修正也不会令轮子反转。
         * straight_2 继续使用同一角度反馈，误差小时趋于四轮同速；
         * 切段不突变清零转向量，也不把弯道起点重新设为零。 */
        if (!SetRouteCommand(forward, turn)) return false;
        s_turn_rpm = turn;
        s_route_update_ms = now;
    }
    if (segment_changed) PrintSegment();
    return true;
}

static void PrintRoute(void)
{
    char line[128];
    uint32_t elapsed = (uint32_t)(HAL_GetTick() - s_start_ms);
    int n = snprintf(line, sizeof(line),
        "ROUTE phase=%s elapsed_ms=%lu yaw_x10=%ld\r\n",
        RoutePhaseName(), (unsigned long)elapsed, RouteYawX10());
    if (n > 0 && (size_t)n < sizeof(line)) Send(line);
}

static void PrintRouteEnd(uint32_t stopped_ms)
{
    /* elapsed_ms 为开始运动到撤销驱动的时间；error_x10 为目标角减估计角。
     * 此处不会等车辆完全滑行停止后再测量，也没有车头真实角度传感器。 */
    char line[160];
    int n = snprintf(line, sizeof(line),
        "ROUTE_END phase=%s elapsed_ms=%lu yaw_x10=%ld error_x10=%ld\r\n",
        RoutePhaseName(), (unsigned long)(uint32_t)(stopped_ms - s_start_ms),
        RouteYawX10(), AngleX10((double)ROBOT_APP_TURN_DEG - s_route_yaw_deg));
    if (n > 0 && (size_t)n < sizeof(line)) Send(line);
}
#endif

static void PrintReady(void)
{
    /* READY表示按键流程已准备好，不表示底盘已开始运动或串口远程已使能。
     * 宏分支会令输出内容对应本次编译选中的本地模式。 */
    char line[320];
#if ROBOT_APP_MODE == 0
    int n = snprintf(line, sizeof(line),
        "READY mode=single-wheel next=%s open_pwm=%d run_ms=%lu; hold USER\r\n",
        s_names[s_test_order[s_next_wheel]], (int)ROBOT_APP_TEST_PWM,
        (unsigned long)ROBOT_APP_RUN_MS);
#elif ROBOT_APP_MODE == 1
    int n = snprintf(line, sizeof(line),
        "READY mode=click movement=%s route=straight_arc_left_straight target_x10=%ld start_x10=%ld ramp_ms=%lu first_ms=%lu turn_x10=%ld delay_ms=%lu run_ms=%lu; click USER\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (long)(ROBOT_APP_START_RPM * 10.0f),
        (unsigned long)ROBOT_APP_RAMP_MS,
        (unsigned long)ROBOT_APP_FIRST_STRAIGHT_MS,
        (long)(ROBOT_APP_TURN_DEG * 10.0f),
        (unsigned long)ROBOT_APP_START_DELAY_MS,
        (unsigned long)ROBOT_APP_RUN_MS);
#else
    int n = snprintf(line, sizeof(line),
        "READY mode=click movement=%s target_x10=%ld delay_ms=%lu run_ms=%lu; click USER\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (unsigned long)ROBOT_APP_START_DELAY_MS,
        (unsigned long)ROBOT_APP_RUN_MS);
#endif
    if (n > 0 && (size_t)n < sizeof(line)) Send(line);
}

static void PrintRunning(void)
{
    /* 记录命令接受时的运行参数；真正转速继续看随后每轮的rpm_x10反馈。 */
    char line[320];
#if ROBOT_APP_MODE == 0
    int n = snprintf(line, sizeof(line),
        "RUN mode=single-wheel wheel=%s open_pwm=%d run_ms=%lu; release USER to stop\r\n",
        s_names[s_active_wheel], (int)ROBOT_APP_TEST_PWM,
        (unsigned long)ROBOT_APP_RUN_MS);
#elif ROBOT_APP_MODE == 1
    int n = snprintf(line, sizeof(line),
        "RUN mode=click movement=%s route=straight_arc_left_straight target_x10=%ld start_x10=%ld ramp_ms=%lu first_ms=%lu turn_x10=%ld run_ms=%lu; press USER to stop\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (long)(ROBOT_APP_START_RPM * 10.0f),
        (unsigned long)ROBOT_APP_RAMP_MS,
        (unsigned long)ROBOT_APP_FIRST_STRAIGHT_MS,
        (long)(ROBOT_APP_TURN_DEG * 10.0f),
        (unsigned long)ROBOT_APP_RUN_MS);
#else
    int n = snprintf(line, sizeof(line),
        "RUN mode=click movement=%s target_x10=%ld run_ms=%lu; press USER to stop\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (unsigned long)ROBOT_APP_RUN_MS);
#endif
    if (n > 0 && (size_t)n < sizeof(line)) Send(line);
}

static void Finish(const char *reason)
{
    /* 先撤销驱动，再更新状态或打印；不会保存下一次运动请求。 */
    Motor4_StopAll();
    s_state = WAIT_RELEASE;
    s_last_motion_ms = HAL_GetTick();
#if ROBOT_APP_MODE != 0
    s_delay_ms = 0U;
#endif
#if ROBOT_APP_MODE == 1
    if (s_route_phase != ROUTE_STRAIGHT_1) {
        Motor4Telemetry t[MOTOR4_COUNT];
        Motor4_GetAllTelemetry(t);
        UpdateRouteYaw(t);
        /* 记录撤驱动时的编码器估角，不代表随后滑行停止后的真实角度。 */
        PrintRouteEnd(s_last_motion_ms);
    }
    ResetRoute();
#endif
    char line[128];
    int n = snprintf(line, sizeof(line),
        "STOP reason=%s; release USER and wait for wheels to stop\r\n", reason);
    if (n > 0 && (size_t)n < sizeof(line)) Send(line);
}

#if ROBOT_APP_MODE == 0
/* 保留原逐轮接线测试：按住运行、松手停止，依次A/B/C/D。 */
static void PollHoldTest(uint32_t now, bool pressed,
                         const Motor4Telemetry t[MOTOR4_COUNT])
{
    if (s_state == RUNNING) {
        if (!pressed) Finish("release");
        else if (HasFault(t)) Finish("motor_fault");
        else if ((uint32_t)(now - s_start_ms) >= ROBOT_APP_RUN_MS) Finish("timeout");
        else Motor4_FeedWatchdog();
        return;
    }

    switch (s_state) {
    case WAIT_RELEASE:
        /* 先松手一次才能重新就绪；运行停止后不能一直按住而自动测下一轮。 */
        if (!pressed) { s_key_ms = now; s_state = RELEASE_FILTER; }
        break;
    case RELEASE_FILTER:
        if (pressed) s_state = WAIT_RELEASE;
        else if ((uint32_t)(now - s_key_ms) >= ROBOT_APP_DEBOUNCE_MS &&
                 WheelsStill(now)) {
            s_state = WAIT_PRESS;
            PrintReady();
        }
        break;
    case WAIT_PRESS:
        if (pressed) { s_key_ms = now; s_state = PRESS_FILTER; }
        break;
    case PRESS_FILTER:
        /* 按住模式在“按下已稳定”后即启动，运行中松手则立即Finish。 */
        if (!pressed) s_state = WAIT_PRESS;
        else if ((uint32_t)(now - s_key_ms) >= ROBOT_APP_DEBOUNCE_MS) {
            if (HasFault(t)) Finish("fault_latched_check_wiring_then_reset");
            else if (!StartCommand()) Finish("command_rejected");
            else {
                s_start_ms = HAL_GetTick();
                s_state = RUNNING;
                Motor4_FeedWatchdog();
                PrintRunning();
            }
        }
        break;
    default:
        Finish("invalid_app_state");
        break;
    }
}
#else
static void PrintDelay(void)
{
    /* 预约等待信息在状态切换时只打印一次，不在每轮Poll重复发送。 */
    char line[320];
#if ROBOT_APP_MODE == 1
    int n = snprintf(line, sizeof(line),
        "DELAY mode=click movement=%s route=straight_arc_left_straight target_x10=%ld start_x10=%ld ramp_ms=%lu first_ms=%lu turn_x10=%ld delay_ms=%lu run_ms=%lu; press USER to cancel\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (long)(ROBOT_APP_START_RPM * 10.0f),
        (unsigned long)ROBOT_APP_RAMP_MS,
        (unsigned long)ROBOT_APP_FIRST_STRAIGHT_MS,
        (long)(ROBOT_APP_TURN_DEG * 10.0f),
        (unsigned long)ROBOT_APP_START_DELAY_MS,
        (unsigned long)ROBOT_APP_RUN_MS);
#else
    int n = snprintf(line, sizeof(line),
        "DELAY mode=click movement=%s target_x10=%ld delay_ms=%lu run_ms=%lu; press USER to cancel\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (unsigned long)ROBOT_APP_START_DELAY_MS,
        (unsigned long)ROBOT_APP_RUN_MS);
#endif
    if (n > 0 && (size_t)n < sizeof(line)) Send(line);
}

/* 完整点击才预约一次：按下去抖、松开去抖、延迟、运行。
 * 取消/停机用原始按下电平立即处理，不再等待按键去抖。
 * 所有计时为非阻塞的uint32模差，不使用HAL_Delay。 */
static void PollClickDrive(uint32_t now, bool pressed,
                          const Motor4Telemetry t[MOTOR4_COUNT])
{
    /* 故障检查优先于按键流程；延迟中的预约同样取消，防止故障后自行启动。 */
    if (HasFault(t)) {
        if (s_state != WAIT_RELEASE) Finish("motor_fault");
        return;
    }

    switch (s_state) {
    case WAIT_RELEASE:
        /* 必须先松开再重新按；复位时一直按住不会被当成有效点击。 */
        if (!pressed) { s_key_ms = now; s_state = RELEASE_FILTER; }
        break;
    case RELEASE_FILTER:
        if (pressed) s_state = WAIT_RELEASE;
        else if ((uint32_t)(now - s_key_ms) >= ROBOT_APP_DEBOUNCE_MS &&
                 WheelsStill(now)) {
            s_state = WAIT_PRESS;
            PrintReady();
        }
        break;
    case WAIT_PRESS:
        if (pressed) { s_key_ms = now; s_state = PRESS_FILTER; }
        break;
    case PRESS_FILTER:
        if (!pressed) s_state = WAIT_PRESS;
        else if ((uint32_t)(now - s_key_ms) >= ROBOT_APP_DEBOUNCE_MS) {
            s_state = WAIT_CLICK_RELEASE;
        }
        break;
    case WAIT_CLICK_RELEASE:
        /* 与模式0不同：点击模式必须等到“按下后再松开”才安排启动。 */
        if (!pressed) { s_key_ms = now; s_state = CLICK_RELEASE_FILTER; }
        break;
    case CLICK_RELEASE_FILTER:
        /* 松开也要稳定30ms；抖回按下时回到等待松开，避免误发一次启动。 */
        if (pressed) s_state = WAIT_CLICK_RELEASE;
        else if ((uint32_t)(now - s_key_ms) >= ROBOT_APP_DEBOUNCE_MS) {
            Motor4_StopAll();
            s_delay_ms = HAL_GetTick();
            s_state = START_DELAY;
            PrintDelay();
        }
        break;
    case START_DELAY:
        /* 延迟阶段不喂运行看门狗，也不发运动目标；再按就取消本次预约。 */
        if (pressed) Finish("delay_cancelled");
        else if ((uint32_t)(now - s_delay_ms) >= ROBOT_APP_START_DELAY_MS) {
            if (!WheelsStill(now)) Finish("wheels_not_still");
            else if (!StartCommand()) Finish("command_rejected");
            else {
                /* 从首次运动命令被接受时计运行总时长，不包含预约延迟。 */
                s_start_ms = HAL_GetTick();
#if ROBOT_APP_MODE == 1
                s_route_update_ms = s_start_ms;
                s_ramp_complete = false;
#endif
                s_state = RUNNING;
                Motor4_FeedWatchdog();
                PrintRunning();
            }
        }
        break;
    case RUNNING:
        /* 先处理按钮/总时限，再更新路线；转角未完成也不能延长 12s 总时限。 */
        if (pressed) Finish("user_stop");
#if ROBOT_APP_MODE == 1
        else if ((uint32_t)(now - s_start_ms) >= ROBOT_APP_RUN_MS) {
            Finish(s_route_phase == ROUTE_ARC_LEFT ? "timeout_arc_incomplete" : "timeout");
        }
        else if (!UpdateRoute(now, t)) Finish("command_rejected");
#else
        else if ((uint32_t)(now - s_start_ms) >= ROBOT_APP_RUN_MS) Finish("timeout");
#endif
        /* 只有仍允许运动且目标更新成功才喂狗。若主循环长时间阻塞，
         * 电机层可在控制中断中检测 500ms 未喂狗并撤销四路驱动。 */
        else Motor4_FeedWatchdog();
        break;
    default:
        Finish("invalid_app_state");
        break;
    }
}
#endif

void RobotApp_Init(void)
{
    /* 在 motor4 和外设初始化之后调用；先明确停止，再准备状态与位置基准。 */
    Motor4_StopAll();
    s_state = WAIT_RELEASE;
#if ROBOT_APP_MODE == 0
    s_next_wheel = 0U;
    s_active_wheel = (unsigned)MOTOR4_FL;
#else
    s_delay_ms = 0U;
#endif
    s_key_ms = HAL_GetTick();
    s_start_ms = s_key_ms;
    s_last_motion_ms = s_key_ms;
    s_print_ms = s_key_ms;
#if ROBOT_APP_MODE == 1
    ResetRoute();
#endif

    Motor4Telemetry t[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(t);
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) s_last_position[i] = t[i].position;

#if ROBOT_APP_MODE == 0
    Send("WAIT mode=single-wheel; release USER and wait for wheels to stop\r\n");
#else
    Send("WAIT mode=click; release USER and wait for wheels to stop\r\n");
#endif
}

void RobotApp_Poll(void)
{
    uint32_t now = HAL_GetTick();
    /* NUCLEO 的蓝色 USER（PC13）按下读到低电平：RESET 是电平 0，
     * 这里不是在调用复位功能；不要与板上的黑色 RESET 按钮混淆。 */
    bool pressed = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
    Motor4Telemetry telemetry[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(telemetry);

    /* delta 是最近一个 10ms 控制周期的变化量，position 是累计计数；
     * 两者都检查，使停止后的滑行或两次 Poll 之间的位置变化也能被发现。 */
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (telemetry[i].delta != 0 ||
            telemetry[i].position != s_last_position[i]) {
            s_last_motion_ms = now;
        }
        s_last_position[i] = telemetry[i].position;
    }

    /* 按键、故障、延迟和运行时限先处理，再做可能耗时的串口发送。 */
#if ROBOT_APP_MODE == 0
    PollHoldTest(now, pressed, telemetry);
#else
    PollClickDrive(now, pressed, telemetry);
#endif

    if ((uint32_t)(now - s_print_ms) >= ROBOT_APP_PRINT_MS) {
        s_print_ms = now;
        Motor4_GetAllTelemetry(telemetry);
#if ROBOT_APP_MODE == 1
        if (s_state == RUNNING) PrintRoute();
#endif
        for (unsigned slot = 0; slot < MOTOR4_COUNT; ++slot) {
            unsigned i = (unsigned)s_test_order[slot];
            char line[128];
            /* 仍按实体接口 A/B/C/D 打印，而不是内部 FL/FR/RL/RR 顺序。
             * cnt：累计编码器计数；target_x10/rpm_x10：目标/反馈 RPM ×10；
             * 例如 850 表示 85.0RPM。pwm：带方向的千分比；fault：故障位。
             * cnt 累计值不同本身不代表当前转速不同，应比较 rpm_x10。
             * 内部 position 是 int64_t，这里沿用 long 打印，F401 上为 32 位。 */
            int n = snprintf(line, sizeof(line),
                "%s cnt=%ld target_x10=%ld rpm_x10=%ld pwm=%d fault=%u\r\n",
                s_names[i], (long)telemetry[i].position,
                (long)(telemetry[i].target_rpm * 10.0f),
                (long)(telemetry[i].rpm * 10.0f),
                (int)telemetry[i].pwm, (unsigned)telemetry[i].fault);
            if (n > 0 && (size_t)n < sizeof(line)) Send(line);
        }
    }
}

#endif /* ROBOT_APP_MODE == 7 */
