#include "robot_app.h"
#include "motor4.h"
#include "mecanum.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#if ROBOT_APP_MODE < 0 || ROBOT_APP_MODE > 6
#error "ROBOT_APP_MODE must be 0..6"
#endif

enum {
    WAIT_RELEASE,
    RELEASE_FILTER,
    WAIT_PRESS,
    PRESS_FILTER,
    WAIT_CLICK_RELEASE,
    CLICK_RELEASE_FILTER,
    START_DELAY,
    RUNNING
};

static uint8_t s_state;
#if ROBOT_APP_MODE == 0
static unsigned s_next_wheel;
static unsigned s_active_wheel;
#else
static uint32_t s_delay_ms;
#endif
static uint32_t s_key_ms;
static uint32_t s_start_ms;
static uint32_t s_last_motion_ms;
static uint32_t s_print_ms;
static int64_t s_last_position[MOTOR4_COUNT];
#if ROBOT_APP_MODE == 1
static uint32_t s_ramp_last_ms;
static bool s_ramp_complete;
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

static void Send(const char *text)
{
    size_t length = strlen(text);
    if (length > 0U && length <= 65535U) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)text,
                              (uint16_t)length, 50U);
    }
}

static bool HasFault(const Motor4Telemetry t[MOTOR4_COUNT])
{
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (t[i].fault != 0U) return true;
    }
    return false;
}

static bool WheelsStill(uint32_t now)
{
    return (uint32_t)(now - s_last_motion_ms) >= 250U;
}

static bool StartCommand(void)
{
#if ROBOT_APP_MODE == 0
    s_active_wheel = (unsigned)s_test_order[s_next_wheel];
    if (!Motor4_SetOpenLoop((Motor4Id)s_active_wheel, ROBOT_APP_TEST_PWM)) {
        return false;
    }
    s_next_wheel = (s_next_wheel + 1U) % MOTOR4_COUNT;
    return true;
#elif ROBOT_APP_MODE == 1
    return Mecanum_SetMixRPM(ROBOT_APP_START_RPM, 0.0f, 0.0f);
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
/* 每10ms最多更新一次同向目标；终点只发送一次，不重置PI状态。 */
static bool UpdateForwardRamp(uint32_t now)
{
    if (s_ramp_complete) return true;
    uint32_t elapsed = (uint32_t)(now - s_start_ms);
    if (elapsed >= ROBOT_APP_RAMP_MS) {
        if (!Mecanum_SetMixRPM(ROBOT_APP_TEST_RPM, 0.0f, 0.0f)) return false;
        s_ramp_complete = true;
        return true;
    }
    if ((uint32_t)(now - s_ramp_last_ms) < 10U) return true;
    float target = ROBOT_APP_START_RPM +
        (ROBOT_APP_TEST_RPM - ROBOT_APP_START_RPM) *
        ((float)elapsed / (float)ROBOT_APP_RAMP_MS);
    if (!Mecanum_SetMixRPM(target, 0.0f, 0.0f)) return false;
    s_ramp_last_ms = now;
    return true;
}
#endif

static void PrintReady(void)
{
    char line[224];
#if ROBOT_APP_MODE == 0
    int n = snprintf(line, sizeof(line),
        "READY mode=single-wheel next=%s open_pwm=%d run_ms=%lu; hold USER\r\n",
        s_names[s_test_order[s_next_wheel]], (int)ROBOT_APP_TEST_PWM,
        (unsigned long)ROBOT_APP_RUN_MS);
#elif ROBOT_APP_MODE == 1
    int n = snprintf(line, sizeof(line),
        "READY mode=click movement=%s target_x10=%ld start_x10=%ld ramp_ms=%lu delay_ms=%lu run_ms=%lu; click USER\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (long)(ROBOT_APP_START_RPM * 10.0f),
        (unsigned long)ROBOT_APP_RAMP_MS,
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
    char line[224];
#if ROBOT_APP_MODE == 0
    int n = snprintf(line, sizeof(line),
        "RUN mode=single-wheel wheel=%s open_pwm=%d run_ms=%lu; release USER to stop\r\n",
        s_names[s_active_wheel], (int)ROBOT_APP_TEST_PWM,
        (unsigned long)ROBOT_APP_RUN_MS);
#elif ROBOT_APP_MODE == 1
    int n = snprintf(line, sizeof(line),
        "RUN mode=click movement=%s target_x10=%ld start_x10=%ld ramp_ms=%lu run_ms=%lu; press USER to stop\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (long)(ROBOT_APP_START_RPM * 10.0f),
        (unsigned long)ROBOT_APP_RAMP_MS,
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
    s_ramp_complete = true;
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
    char line[224];
#if ROBOT_APP_MODE == 1
    int n = snprintf(line, sizeof(line),
        "DELAY mode=click movement=%s target_x10=%ld start_x10=%ld ramp_ms=%lu delay_ms=%lu run_ms=%lu; press USER to cancel\r\n",
        MOVEMENT_NAME, (long)(ROBOT_APP_TEST_RPM * 10.0f),
        (long)(ROBOT_APP_START_RPM * 10.0f),
        (unsigned long)ROBOT_APP_RAMP_MS,
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
    if (HasFault(t)) {
        if (s_state != WAIT_RELEASE) Finish("motor_fault");
        return;
    }

    switch (s_state) {
    case WAIT_RELEASE:
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
        if (!pressed) { s_key_ms = now; s_state = CLICK_RELEASE_FILTER; }
        break;
    case CLICK_RELEASE_FILTER:
        if (pressed) s_state = WAIT_CLICK_RELEASE;
        else if ((uint32_t)(now - s_key_ms) >= ROBOT_APP_DEBOUNCE_MS) {
            Motor4_StopAll();
            s_delay_ms = HAL_GetTick();
            s_state = START_DELAY;
            PrintDelay();
        }
        break;
    case START_DELAY:
        if (pressed) Finish("delay_cancelled");
        else if ((uint32_t)(now - s_delay_ms) >= ROBOT_APP_START_DELAY_MS) {
            if (!WheelsStill(now)) Finish("wheels_not_still");
            else if (!StartCommand()) Finish("command_rejected");
            else {
                /* 从运动命令被接受时计10秒，预约等待时间不包含在内。 */
                s_start_ms = HAL_GetTick();
#if ROBOT_APP_MODE == 1
                s_ramp_last_ms = s_start_ms;
                s_ramp_complete = false;
#endif
                s_state = RUNNING;
                Motor4_FeedWatchdog();
                PrintRunning();
            }
        }
        break;
    case RUNNING:
        if (pressed) Finish("user_stop");
        else if ((uint32_t)(now - s_start_ms) >= ROBOT_APP_RUN_MS) Finish("timeout");
#if ROBOT_APP_MODE == 1
        else if (!UpdateForwardRamp(now)) Finish("command_rejected");
#endif
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
    s_ramp_last_ms = s_key_ms;
    s_ramp_complete = true;
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
    bool pressed = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
    Motor4Telemetry telemetry[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(telemetry);

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
        for (unsigned slot = 0; slot < MOTOR4_COUNT; ++slot) {
            unsigned i = (unsigned)s_test_order[slot];
            char line[128];
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
