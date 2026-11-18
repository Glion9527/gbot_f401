#include "link_v2.h"
#include "link_v2_contract.h"
#include "motor4.h"
#include "robot_app.h"

#include <math.h>
#include <stddef.h>

/* 接收环形缓冲：512 字节足以容纳主循环忙时到达的多帧；与 v1 同容量。 */
#define L2_RX_CAPACITY 512U
/* ARM 前要求四轮已停止且最近 250ms 无编码器运动，和 v1 一致。 */
#define L2_ARM_QUIET_MS 250U
#define L2_PI 3.14159265358979323846f

static volatile uint8_t s_rx[L2_RX_CAPACITY];
static volatile uint32_t s_rx_at[L2_RX_CAPACITY];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static uint8_t s_rx_byte;

static volatile bool s_ready;
static volatile bool s_armed;
static volatile bool s_estop;
static volatile bool s_timed_out;
static volatile uint16_t s_last_seq;
static volatile uint32_t s_valid_at;
static volatile uint32_t s_generation;

static uint32_t s_last_motion_ms;
static uint32_t s_telem_due_ms;
static int64_t s_position[MOTOR4_COUNT];
static int32_t s_delta_acc[MOTOR4_COUNT];

/* 解析状态只由主循环修改；ISR 通过 s_generation 让半帧失效，
 * 不在中断里直接清解析长度，避免和主循环竞争。 */
static uint8_t s_frame[LINK_V2_CMD_SIZE];
static size_t s_frame_len;
static uint32_t s_frame_at;
static uint32_t s_frame_generation;

/* TELEM 发送缓冲必须是静态的：HAL_UART_Transmit_IT 在函数返回后仍会读取它。 */
static uint8_t s_tx_frame[LINK_V2_TELEM_SIZE];
static volatile bool s_tx_busy;

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

static uint16_t GetU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static int16_t GetI16(const uint8_t *p)
{
    return (int16_t)GetU16(p);
}

static void PutU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static int16_t ClampI16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

/* 遥控模式下 USER 键只做本地停止；低电平表示按下。 */
static bool ButtonPressed(void)
{
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
}

static uint8_t FaultMask(const Motor4Telemetry t[MOTOR4_COUNT])
{
    uint8_t faults = 0U;
    for (unsigned i = 0U; i < MOTOR4_COUNT; ++i) {
        faults |= t[i].fault;
    }
    return faults;
}

/* 撤销运动许可：停四轮、丢弃环形缓冲、让正在解析的半帧失效。
 * 不重置 s_last_seq，重放/旧序号仍然会被拒绝。 */
static void DisarmLocked(void)
{
    Motor4_StopAll();
    s_armed = false;
    s_tail = s_head;
    ++s_generation;
}

static void BadFrame(void)
{
    uint32_t key = Lock();
    if (s_armed) {
        DisarmLocked();
    }
    Unlock(key);
}

static void PushByte(uint8_t byte, uint32_t at)
{
    uint32_t key = Lock();
    uint16_t next = (uint16_t)((s_head + 1U) % L2_RX_CAPACITY);
    if (next == s_tail) {
        /* 队列溢出说明主循环已经跟不上：立即停机，清空缓冲重新同步。 */
        DisarmLocked();
        s_head = 0U;
        s_tail = 0U;
    } else {
        s_rx[s_head] = byte;
        s_rx_at[s_head] = at;
        s_head = next;
    }
    Unlock(key);
}

/* 已通过长度/CRC/版本/类型/seq 检查的一帧 CMD。 */
static void ExecuteFrame(const uint8_t frame[LINK_V2_CMD_SIZE], uint32_t generation)
{
    const uint16_t seq = GetU16(&frame[LINK_V2_CMD_OFF_SEQ]);
    const uint16_t flags = GetU16(&frame[LINK_V2_CMD_OFF_FLAGS]);
    int16_t wheel[LINK_V2_WHEEL_COUNT];
    for (unsigned i = 0U; i < LINK_V2_WHEEL_COUNT; ++i) {
        wheel[i] = GetI16(&frame[LINK_V2_CMD_OFF_WHEELS + i * 2U]);
    }

    uint32_t key = Lock();
    if (generation != s_generation) {
        Unlock(key);
        return;
    }

    /* u16 回绕比较：只接受比上次更新的序号；重复/旧序号一律忽略。 */
    const uint16_t distance = (uint16_t)(seq - s_last_seq);
    if (distance == 0U || distance >= 0x8000U) {
        Unlock(key);
        return;
    }
    s_last_seq = seq;

    Motor4Telemetry t[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(t);
    const uint8_t faults = FaultMask(t);
    const uint32_t now = HAL_GetTick();

    if ((flags & LINK_V2_FLAG_ESTOP) != 0U) {
        s_estop = true;
        DisarmLocked();
        Unlock(key);
        return;
    }
    if ((flags & LINK_V2_FLAG_DISARM) != 0U) {
        DisarmLocked();
        Unlock(key);
        return;
    }

    if (!s_armed) {
        if ((flags & LINK_V2_FLAG_ARM) == 0U) {
            bool nonzero = false;
            for (unsigned i = 0U; i < LINK_V2_WHEEL_COUNT; ++i) {
                if (wheel[i] != 0) {
                    nonzero = true;
                }
            }
            if (nonzero) {
                DisarmLocked();
                Unlock(key);
                return;
            }
            (void)Motor4_SetAllSpeedsRPM(0.0f, 0.0f, 0.0f, 0.0f);
            Unlock(key);
            return;
        }

        /* ARM 条件：无故障、USER 未按下、四轮 STOP 且最近 250ms 无运动。 */
        bool quiet = (uint32_t)(now - s_last_motion_ms) >= L2_ARM_QUIET_MS;
        for (unsigned i = 0U; i < MOTOR4_COUNT; ++i) {
            if (t[i].mode != MOTOR4_MODE_STOP || t[i].delta != 0) {
                quiet = false;
            }
        }
        if (faults != 0U || ButtonPressed() || !quiet) {
            DisarmLocked();
            Unlock(key);
            return;
        }
        s_armed = true;
        s_valid_at = now;
        s_timed_out = false;
        s_estop = false;
    }

    /* mm/s -> 输出轴 RPM。超过 motor4 的硬上限就拒绝，不把目标交给 PI。 */
    const float factor = 60.0f / (L2_PI * ROBOT_APP_WHEEL_DIAMETER_MM);
    float rpm[LINK_V2_WHEEL_COUNT];
    bool invalid = false;
    for (unsigned i = 0U; i < LINK_V2_WHEEL_COUNT; ++i) {
        rpm[i] = (float)wheel[i] * factor;
        if (!isfinite(rpm[i]) || fabsf(rpm[i]) > MOTOR4_MAX_TARGET_RPM) {
            invalid = true;
        }
    }
    if (invalid || !Motor4_SetAllSpeedsRPM(rpm[0], rpm[1], rpm[2], rpm[3])) {
        DisarmLocked();
        Unlock(key);
        return;
    }
    s_valid_at = now;
    Unlock(key);
}

void LinkV2_Tick10ms(void)
{
    if (!s_ready) {
        return;
    }

    uint32_t key = Lock();
    const uint32_t now = HAL_GetTick();
    Motor4Telemetry t[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(t);

    for (unsigned i = 0U; i < MOTOR4_COUNT; ++i) {
        if (t[i].delta != 0 || t[i].position != s_position[i]) {
            s_last_motion_ms = now;
        }
        s_position[i] = t[i].position;
        s_delta_acc[i] += (int32_t)t[i].delta;
    }

    if (s_armed && (ButtonPressed() || FaultMask(t) != 0U ||
                    (uint32_t)(now - s_valid_at) >= LINK_V2_CMD_TIMEOUT_MS)) {
        if ((uint32_t)(now - s_valid_at) >= LINK_V2_CMD_TIMEOUT_MS) {
            s_timed_out = true;
        }
        DisarmLocked();
    }
    Unlock(key);
}

static void SendTelemetry(void)
{
    uint32_t key = Lock();
    const bool armed = s_armed;
    const bool estop = s_estop;
    const bool timed_out = s_timed_out;
    const uint16_t seq = s_last_seq;
    int32_t delta[MOTOR4_COUNT];
    for (unsigned i = 0U; i < MOTOR4_COUNT; ++i) {
        delta[i] = s_delta_acc[i];
        s_delta_acc[i] = 0;
    }
    Unlock(key);

    Motor4Telemetry t[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(t);
    uint8_t faults = 0U;
    for (unsigned i = 0U; i < MOTOR4_COUNT; ++i) {
        faults |= t[i].fault;
    }

    uint16_t status = 0U;
    if (armed) {
        status |= LINK_V2_STATUS_ARMED;
    }
    if (faults != 0U) {
        status |= LINK_V2_STATUS_FAULT;
    }
    if (estop) {
        status |= LINK_V2_STATUS_ESTOP;
    }
    if (timed_out) {
        status |= LINK_V2_STATUS_CMD_TIMEOUT;
    }

    s_tx_frame[LINK_V2_TELEM_OFF_HEADER0] = LINK_V2_HEADER0;
    s_tx_frame[LINK_V2_TELEM_OFF_HEADER1] = LINK_V2_HEADER1;
    s_tx_frame[LINK_V2_TELEM_OFF_VERSION] = LINK_V2_VERSION;
    s_tx_frame[LINK_V2_TELEM_OFF_TYPE] = LINK_V2_TYPE_TELEM;
    PutU16(&s_tx_frame[LINK_V2_TELEM_OFF_SEQ], seq);
    PutU16(&s_tx_frame[LINK_V2_TELEM_OFF_MCU_MS], (uint16_t)HAL_GetTick());
    PutU16(&s_tx_frame[LINK_V2_TELEM_OFF_STATUS], status);
    PutU16(&s_tx_frame[LINK_V2_TELEM_OFF_FAULT], (uint16_t)faults);

    for (unsigned i = 0U; i < MOTOR4_COUNT; ++i) {
        PutU16(&s_tx_frame[LINK_V2_TELEM_OFF_DELTA + i * 2U],
               (uint16_t)ClampI16(delta[i]));
        const int32_t rpm10 = (int32_t)(t[i].rpm * 10.0f);
        PutU16(&s_tx_frame[LINK_V2_TELEM_OFF_RPM10 + i * 2U],
               (uint16_t)ClampI16(rpm10));
    }

    const uint16_t crc = link_v2_crc16(
        &s_tx_frame[LINK_V2_TELEM_OFF_VERSION],
        LINK_V2_TELEM_OFF_CRC - LINK_V2_TELEM_OFF_VERSION);
    PutU16(&s_tx_frame[LINK_V2_TELEM_OFF_CRC], crc);

    /* 上一帧还在发送就丢弃本帧；遥测可以丢，命令不能丢。 */
    if (s_tx_busy) {
        return;
    }
    if (HAL_UART_Transmit_IT(&huart2, s_tx_frame,
                             (uint16_t)LINK_V2_TELEM_SIZE) == HAL_OK) {
        s_tx_busy = true;
    }
}

static void ParserByte(uint8_t byte, uint32_t at)
{
    if (s_frame_len == 0U) {
        if (byte == LINK_V2_HEADER0) {
            s_frame[0] = byte;
            s_frame_at = at;
            s_frame_generation = s_generation;
            s_frame_len = 1U;
        }
        return;
    }

    if (s_frame_len == 1U) {
        if (byte == LINK_V2_HEADER1) {
            s_frame[1] = byte;
            s_frame_len = 2U;
        } else if (byte == LINK_V2_HEADER0) {
            /* 重复的 A5：把它当作新的帧头，不丢同步。 */
            s_frame[0] = byte;
            s_frame_at = at;
            s_frame_generation = s_generation;
        } else {
            s_frame_len = 0U;
        }
        return;
    }

    s_frame[s_frame_len] = byte;
    ++s_frame_len;
    if (s_frame_len < (size_t)LINK_V2_CMD_SIZE) {
        return;
    }
    s_frame_len = 0U;

    if (s_frame_generation != s_generation) {
        return;
    }
    if (s_frame[LINK_V2_CMD_OFF_VERSION] != LINK_V2_VERSION ||
        s_frame[LINK_V2_CMD_OFF_TYPE] != LINK_V2_TYPE_CMD) {
        BadFrame();
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_frame_at) >= LINK_V2_CMD_TIMEOUT_MS) {
        BadFrame();
        return;
    }
    const uint16_t given = GetU16(&s_frame[LINK_V2_CMD_OFF_CRC]);
    const uint16_t actual = link_v2_crc16(
        &s_frame[LINK_V2_CMD_OFF_VERSION],
        LINK_V2_CMD_OFF_CRC - LINK_V2_CMD_OFF_VERSION);
    if (given != actual) {
        BadFrame();
        return;
    }
    ExecuteFrame(s_frame, s_frame_generation);
}

void LinkV2_Poll(void)
{
    if (!s_ready) {
        return;
    }

    for (unsigned processed = 0U; processed < L2_RX_CAPACITY; ++processed) {
        uint32_t key = Lock();
        if (s_tail == s_head) {
            Unlock(key);
            break;
        }
        const uint8_t byte = s_rx[s_tail];
        const uint32_t at = s_rx_at[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) % L2_RX_CAPACITY);
        Unlock(key);
        ParserByte(byte, at);
    }

    uint32_t key = Lock();
    const uint32_t now = HAL_GetTick();
    /* 300ms 许可有效且主循环仍在运行时才喂电机看门狗；
     * Tick 只负责撤销，不能替主循环持续喂狗。 */
    if (s_armed && !ButtonPressed() &&
        (uint32_t)(now - s_valid_at) < LINK_V2_CMD_TIMEOUT_MS) {
        Motor4_FeedWatchdog();
    }

    bool send = false;
    if ((int32_t)(now - s_telem_due_ms) >= 0) {
        s_telem_due_ms = now + LINK_V2_TELEM_PERIOD_MS;
        send = true;
    }
    Unlock(key);

    if (send) {
        SendTelemetry();
    }
}

HAL_StatusTypeDef LinkV2_Init(void)
{
    uint32_t key = Lock();
    s_ready = false;
    s_armed = false;
    s_estop = false;
    s_timed_out = false;
    s_head = 0U;
    s_tail = 0U;
    s_frame_len = 0U;
    s_last_seq = 0U;
    s_generation = 0U;
    s_valid_at = HAL_GetTick();
    s_last_motion_ms = s_valid_at;
    s_telem_due_ms = s_valid_at;
    s_tx_busy = false;
    for (unsigned i = 0U; i < MOTOR4_COUNT; ++i) {
        s_position[i] = 0;
        s_delta_acc[i] = 0;
    }
    Motor4_StopAll();

    /* 在临界区内启动接收：若第一个字节在 s_ready 置位前到达，
     * 回调会直接返回且不再挂起接收，链路就悄悄停住了。 */
    const HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
    s_ready = (status == HAL_OK);
    Unlock(key);
    return status;
}

void LinkV2_UART_RxCplt(UART_HandleTypeDef *uart)
{
    if (uart != &huart2 || !s_ready) {
        return;
    }
    PushByte(s_rx_byte, HAL_GetTick());
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        LinkV2_UART_Error(uart);
    }
}

void LinkV2_UART_Error(UART_HandleTypeDef *uart)
{
    if (uart != &huart2 || !s_ready) {
        return;
    }
    uint32_t key = Lock();
    DisarmLocked();
    s_tx_busy = false;
    Unlock(key);

    (void)HAL_UART_AbortReceive_IT(&huart2);
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        key = Lock();
        s_ready = false;
        Unlock(key);
    }
}

void LinkV2_UART_TxCplt(UART_HandleTypeDef *uart)
{
    if (uart == &huart2) {
        s_tx_busy = false;
    }
}

#ifdef LINK_V2_HOST_TEST
void LinkV2_TestReset(void)
{
    (void)LinkV2_Init();
}

void LinkV2_TestFeed(const uint8_t *data, size_t len)
{
    for (size_t i = 0U; i < len; ++i) {
        PushByte(data[i], HAL_GetTick());
    }
}

bool LinkV2_TestArmed(void)
{
    return s_armed;
}

uint16_t LinkV2_TestLastSeq(void)
{
    return s_last_seq;
}
#endif
