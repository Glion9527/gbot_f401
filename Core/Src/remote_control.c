#include "remote_control.h"
#include "motor4.h"
#include "mecanum.h"
#include "robot_app.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define RX_CAPACITY        512U
#define COMMAND_LIMIT      127U
#define COMMAND_TIMEOUT_MS 300U
#define ARM_QUIET_MS       250U
#define TELEMETRY_MS       100U
#define REMOTE_MAX_RPM     60.0f

/* 单生产者（USART2 IRQ）/单消费者（主循环）缓冲；时间戳防止积压旧帧
 * 在主循环恢复后被当成刚收到的新运动指令。所有共享状态均用短临界区保护。 */
static uint8_t s_rx_byte;
static volatile uint8_t s_rx[RX_CAPACITY];
static volatile uint32_t s_rx_at[RX_CAPACITY];
static volatile uint16_t s_head, s_tail;
static volatile bool s_drop_rx;
static volatile bool s_ready;
static volatile bool s_armed;
static volatile uint32_t s_session, s_last_seq, s_valid_at;
static volatile uint32_t s_generation;
static uint32_t s_line_generation;
static char s_line[COMMAND_LIMIT + 1U];
static size_t s_line_length;
static uint32_t s_line_at;
static uint32_t s_last_motion_ms, s_print_ms;
static int64_t s_position[MOTOR4_COUNT];

static uint32_t Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static void Unlock(uint32_t key) { __set_PRIMASK(key); }

static bool ButtonPressed(void)
{
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
}

/* 不自动清 motor4 故障，不自动恢复运动。丢弃待处理字节及半帧，
 * 防止一次停机之后继续执行先前排队的 ARM/VEL。调用者持有临界区。 */
static void DisarmLocked(void)
{
    Motor4_StopAll();
    s_armed = false;
    s_tail = s_head;
    ++s_generation;
}

static uint8_t FaultMask(const Motor4Telemetry t[MOTOR4_COUNT])
{
    uint8_t faults = 0U;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) faults |= t[i].fault;
    return faults;
}

static uint16_t CRC16(const char *data, size_t size)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)(uint8_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8U; ++bit) {
            crc = (uint16_t)((crc & 0x8000U) != 0U ?
                ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
        }
    }
    return crc;
}

static void BadFrame(void)
{
    uint32_t key = Lock();
    if (s_armed) DisarmLocked();
    Unlock(key);
}

/* 主循环同步发送，USART 接收中断仍开启；控制中断能够抢占发送。
 * 发送错误也撤销使能，不能让单向失联的客户端继续控制旧目标。 */
static void SendPayload(const char *payload)
{
    char frame[512];
    size_t size = strlen(payload);
    if (size + 7U >= sizeof(frame)) { BadFrame(); return; }
    int n = snprintf(frame, sizeof(frame), "@%s*%04X\n", payload,
                     (unsigned)CRC16(payload, size));
    if (n <= 0 || (size_t)n >= sizeof(frame) ||
        HAL_UART_Transmit(&huart2, (uint8_t *)frame, (uint16_t)n, 50U) != HAL_OK) {
        BadFrame();
    }
}

static void Ack(uint32_t session, uint32_t seq, const char *result)
{
    char payload[80];
    int n = snprintf(payload, sizeof(payload), "ACK,%lu,%lu,%s",
                     (unsigned long)session, (unsigned long)seq, result);
    if (n > 0 && (size_t)n < sizeof(payload)) SendPayload(payload);
}

/* 严格十进制：不接受空白、+号、指数、溢出或尾随字符。 */
static bool ParseU32(const char *text, uint32_t *out)
{
    if (*text == '\0') return false;
    uint32_t value = 0U;
    for (; *text != '\0'; ++text) {
        if (*text < '0' || *text > '9') return false;
        uint32_t digit = (uint32_t)(*text - '0');
        if (value > (UINT32_MAX - digit) / 10U) return false;
        value = value * 10U + digit;
    }
    *out = value;
    return true;
}

static bool ParseAxis(const char *text, int32_t limit, int32_t *out)
{
    bool negative = *text == '-';
    if (negative) ++text;
    uint32_t magnitude;
    if (!ParseU32(text, &magnitude) || magnitude > (uint32_t)limit) return false;
    *out = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return true;
}

static bool NewSequence(uint32_t seq)
{
    uint32_t distance = seq - s_last_seq;
    return distance != 0U && distance < 0x80000000U;
}

/* 一次校验四轮估算峰值，再按同一比例降低整个车身指令，保持方向/曲率。
 * 限制的是轮速目标，不是硬件电流；60RPM 是此阶段遥控的调速上限。 */
static bool SetBodyVelocity(int32_t vx, int32_t vy, int32_t wz)
{
    const float radius = ROBOT_APP_WHEEL_DIAMETER_MM / 2000.0f;
    const float half_l = ROBOT_APP_WHEELBASE_MM / 2000.0f;
    const float half_w = ROBOT_APP_TRACK_MM / 2000.0f;
    float f = (float)vx / 1000.0f;
    float l = (float)vy / 1000.0f;
    float t = (float)wz / 1000.0f;
    const float rpm_factor = 60.0f / (2.0f * 3.14159265358979323846f * radius);
    float rotation = (half_l + half_w) * t;
    float wheel[4] = { f-l-rotation, f+l+rotation, f+l-rotation, f-l+rotation };
    float peak = 0.0f;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        float rpm = fabsf(wheel[i]) * rpm_factor;
        if (rpm > peak) peak = rpm;
    }
    float scale = peak > REMOTE_MAX_RPM ? REMOTE_MAX_RPM / peak : 1.0f;
    return Mecanum_SetBodyVelocity(f * scale, l * scale, t * scale,
                                  radius, half_l, half_w);
}

static void Execute(char *payload, uint32_t generation)
{
    char *part[7];
    unsigned count = 1U;
    part[0] = payload;
    for (char *p = payload; *p != '\0'; ++p) {
        if (*p == ',') {
            if (count == 7U) { BadFrame(); return; }
            *p = '\0';
            part[count++] = p + 1;
        }
    }
    uint32_t session, seq;
    if (count < 3U || strlen(part[0]) != 1U ||
        !ParseU32(part[1], &session) || session == 0U ||
        !ParseU32(part[2], &seq)) { BadFrame(); return; }

    char type = part[0][0];
    bool structure_ok = (type == 'V' && count == 6U) ||
        ((type == 'H' || type == 'A' || type == 'S') && count == 3U);
    if (!structure_ok) { BadFrame(); Ack(session, seq, "INVALID"); return; }

    Motor4Telemetry t[MOTOR4_COUNT];
    const char *result = "OK";
    uint32_t key = Lock();
    /* Tick/接收错误可能恰好在CRC解析期间撤销许可；旧半帧不得越过该边界。 */
    if (generation != s_generation || !s_ready) {
        Unlock(key);
        return;
    }
    uint32_t now = HAL_GetTick();
    if (type == 'H') {
        if (seq != 0U) result = "INVALID";
        else {
            DisarmLocked();
            s_session = session;
            s_last_seq = 0U;
        }
    } else if (session != s_session || !NewSequence(seq)) {
        /* 重播/旧session不刷新任何许可，也不覆盖当前会话的序号。 */
        result = "INVALID";
    } else {
        s_last_seq = seq;
        Motor4_GetAllTelemetry(t);
        if (type == 'S') {
            DisarmLocked();
        } else if (FaultMask(t) != 0U) {
            DisarmLocked();
            result = "FAULT";
        } else if (type == 'A') {
            bool quiet = (uint32_t)(now - s_last_motion_ms) >= ARM_QUIET_MS;
            for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
                if (t[i].mode != MOTOR4_MODE_STOP || t[i].delta != 0) quiet = false;
            }
            if (s_armed || ButtonPressed() || !quiet) result = "BUSY";
            else {
                s_armed = true;
                /* ARM仅提供首次VEL的窗口；再次ARM不会延续旧VEL许可。 */
                s_valid_at = now;
            }
        } else {
            int32_t vx, vy, wz;
            if (!ParseAxis(part[3], 150, &vx) || !ParseAxis(part[4], 150, &vy) ||
                !ParseAxis(part[5], 600, &wz)) {
                DisarmLocked();
                result = "INVALID";
            } else if (!s_armed || ButtonPressed()) {
                DisarmLocked();
                result = "DISARMED";
            } else if ((uint32_t)(now - s_valid_at) >= COMMAND_TIMEOUT_MS) {
                DisarmLocked();
                result = "DISARMED";
            } else if (!SetBodyVelocity(vx, vy, wz)) {
                /* 包括未满足先停稳再反转的情况；不排队等待自动换向。 */
                DisarmLocked();
                result = "REJECTED";
            } else {
                s_valid_at = now;
            }
        }
    }
    Unlock(key);
    Ack(session, seq, result);
}

static void ParseFrame(uint32_t generation)
{
    if (s_line_length < 8U || s_line[0] != '@') { BadFrame(); return; }
    char *star = strchr(s_line, '*');
    if (star == NULL || star + 5 != s_line + s_line_length) { BadFrame(); return; }
    uint16_t given = 0U;
    for (unsigned i = 1U; i <= 4U; ++i) {
        char c = star[i];
        unsigned nibble;
        if (c >= '0' && c <= '9') nibble = (unsigned)(c - '0');
        else if (c >= 'A' && c <= 'F') nibble = (unsigned)(c - 'A') + 10U;
        else { BadFrame(); return; }
        given = (uint16_t)((given << 4) | nibble);
    }
    if (given != CRC16(s_line + 1, (size_t)(star - s_line - 1))) {
        BadFrame();
        return;
    }
    *star = '\0';
    Execute(s_line + 1, generation);
}

/* newlib-nano 的 printf 构建不一定启用 %lld，手动转64位计数，
 * 包括INT64_MIN也不做有符号取负溢出。 */
static void FormatCount(int64_t value, char out[22])
{
    char reverse[20];
    size_t count = 0U, at = 0U;
    uint64_t magnitude = value < 0 ? 0U - (uint64_t)value : (uint64_t)value;
    do { reverse[count++] = (char)('0' + magnitude % 10U); magnitude /= 10U; }
    while (magnitude != 0U);
    if (value < 0) out[at++] = '-';
    while (count != 0U) out[at++] = reverse[--count];
    out[at] = '\0';
}

static long RPMx10(float rpm)
{
    double value = (double)rpm * 10.0;
    if (!isfinite(value)) return 0;
    if (value > (double)INT32_MAX) return (long)INT32_MAX;
    if (value < (double)INT32_MIN) return (long)INT32_MIN;
    return (long)value;
}

static void PrintTelemetry(void)
{
    Motor4Telemetry t[MOTOR4_COUNT];
    uint32_t key = Lock();
    Motor4_GetAllTelemetry(t);
    uint32_t now = HAL_GetTick(), session = s_session, seq = s_last_seq;
    bool armed = s_armed;
    Unlock(key);
    char cnt[MOTOR4_COUNT][22];
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) FormatCount(t[i].position, cnt[i]);
    char payload[384];
    int n = snprintf(payload, sizeof(payload),
        "T,%lu,%lu,%u,%lu,%u,%s,%s,%s,%s,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d",
        (unsigned long)session, (unsigned long)now, armed ? 1U : 0U,
        (unsigned long)seq, (unsigned)FaultMask(t), cnt[0], cnt[1], cnt[2], cnt[3],
        RPMx10(t[0].rpm), RPMx10(t[1].rpm), RPMx10(t[2].rpm), RPMx10(t[3].rpm),
        RPMx10(t[0].target_rpm), RPMx10(t[1].target_rpm),
        RPMx10(t[2].target_rpm), RPMx10(t[3].target_rpm),
        (int)t[0].pwm, (int)t[1].pwm, (int)t[2].pwm, (int)t[3].pwm);
    if (n > 0 && (size_t)n < sizeof(payload)) SendPayload(payload);
    else BadFrame();
}

HAL_StatusTypeDef RemoteControl_Init(void)
{
    uint32_t key = Lock();
    s_ready = false;
    Motor4_StopAll();
    s_armed = false;
    s_session = s_last_seq = 0U;
    s_generation = s_line_generation = 0U;
    s_head = s_tail = 0U;
    s_drop_rx = false;
    s_line_length = 0U;
    s_valid_at = s_last_motion_ms = HAL_GetTick();
    s_print_ms = s_valid_at - TELEMETRY_MS;
    Motor4Telemetry t[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(t);
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) s_position[i] = t[i].position;
    s_ready = true;
    Unlock(key);
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
    if (status != HAL_OK) {
        key = Lock();
        s_ready = false;
        DisarmLocked();
        Unlock(key);
    }
    return status;
}

void RemoteControl_UART_RxCplt(UART_HandleTypeDef *uart)
{
    if (uart != &huart2 || !s_ready) return;
    uint32_t key = Lock();
    uint8_t byte = s_rx_byte;
    uint16_t next = (uint16_t)((s_head + 1U) % RX_CAPACITY);
    if (s_drop_rx) {
        if (byte == '\n') s_drop_rx = false;
    } else if (next == s_tail) {
        DisarmLocked();
        s_drop_rx = byte != '\n';
    } else {
        s_rx[s_head] = byte;
        s_rx_at[s_head] = HAL_GetTick();
        s_head = next;
    }
    Unlock(key);
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        RemoteControl_UART_Error(uart);
    }
}

void RemoteControl_UART_Error(UART_HandleTypeDef *uart)
{
    if (uart != &huart2 || !s_ready) return;
    uint32_t key = Lock();
    DisarmLocked();
    s_drop_rx = false;
    Unlock(key);
    /* 无DMA时AbortReceive_IT立即终止当前接收，再等待新完整@帧。 */
    (void)HAL_UART_AbortReceive_IT(&huart2);
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        key = Lock();
        s_ready = false;
        Unlock(key);
    }
}

void RemoteControl_Tick10ms(void)
{
    uint32_t key = Lock();
    if (!s_ready) { Unlock(key); return; }
    uint32_t now = HAL_GetTick();
    Motor4Telemetry t[MOTOR4_COUNT];
    Motor4_GetAllTelemetry(t);
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) {
        if (t[i].delta != 0 || t[i].position != s_position[i]) s_last_motion_ms = now;
        s_position[i] = t[i].position;
    }
    if (s_armed && (ButtonPressed() || FaultMask(t) != 0U ||
        (uint32_t)(now - s_valid_at) >= COMMAND_TIMEOUT_MS)) {
        DisarmLocked();
    }
    Unlock(key);
}

void RemoteControl_Poll(void)
{
    if (!s_ready) return;
    /* 有界地消费一批字节，防止持续输入霸占主循环。 */
    for (unsigned processed = 0U; processed < RX_CAPACITY; ++processed) {
        uint32_t key = Lock();
        if (s_line_generation != s_generation) {
            s_line_generation = s_generation;
            s_line_length = 0U;
        }
        if (s_tail == s_head) { Unlock(key); break; }
        uint8_t byte = s_rx[s_tail];
        uint32_t arrived = s_rx_at[s_tail];
        uint32_t generation = s_generation;
        s_tail = (uint16_t)((s_tail + 1U) % RX_CAPACITY);
        Unlock(key);

        if (byte == '@') {
            if (s_line_length != 0U) { BadFrame(); continue; }
            s_line_at = arrived;
            s_line[s_line_length++] = '@';
        } else if (byte == '\n') {
            if (s_line_length != 0U) {
                if (s_line[s_line_length - 1U] == '\r') --s_line_length;
                s_line[s_line_length] = '\0';
                if ((uint32_t)(HAL_GetTick() - s_line_at) >= COMMAND_TIMEOUT_MS) BadFrame();
                else ParseFrame(generation);
                s_line_length = 0U;
            }
        } else if (s_line_length != 0U) {
            if (s_line_length >= COMMAND_LIMIT || byte < 0x20U || byte > 0x7EU) {
                /* CR只允许紧靠LF；其余控制字符不能藏在payload中。 */
                if (byte == '\r' && s_line_length < COMMAND_LIMIT) {
                    s_line[s_line_length++] = (char)byte;
                } else { BadFrame(); s_line_length = 0U; }
            } else s_line[s_line_length++] = (char)byte;
        } else if (byte != '\r') {
            BadFrame();
        }
    }
    uint32_t key = Lock();
    uint32_t now = HAL_GetTick();
    if (s_line_length != 0U && (uint32_t)(now - s_line_at) >= COMMAND_TIMEOUT_MS) {
        if (s_armed) DisarmLocked();
        s_line_length = 0U;
    }
    if (s_armed && !ButtonPressed() &&
        (uint32_t)(now - s_valid_at) < COMMAND_TIMEOUT_MS) {
        Motor4_FeedWatchdog();
    }
    bool print = (uint32_t)(now - s_print_ms) >= TELEMETRY_MS;
    if (print) s_print_ms = now;
    Unlock(key);
    if (print) PrintTelemetry();
}
