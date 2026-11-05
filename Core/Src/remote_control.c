#include "remote_control.h"
#include "motor4.h"
#include "mecanum.h"
#include "robot_app.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* 遥控链路在 STM32 这一端的实现：
 *   ROS /cmd_vel -> Ubuntu 网络客户端 -> Mac 网关 -> USART2 -> 本模块。
 * Ubuntu 客户端已把速度换算成整数单位，Mac 按协议编码；本模块只认识 H/A/S/V 串口命令，
 * 不认识 ROS 话题名、TwistStamped 或 JSON。协议两端必须约定相同字段顺序。
 *
 * 执行分工：接收中断保存字节；Poll 在主循环中组帧、校验、解释命令和发送；
 * Tick10ms 在定时中断中检查运行许可。具体 PWM 和轮速 PI 仍交给 motor4。
 * 一帧的格式为 @正文*四位十六进制CRC\n，正文各字段用逗号分隔。 */
/* 缓冲容量大于单帧上限，允许主循环暂忙时保存多个字节；环形队列留一个空槽。 */
#define RX_CAPACITY        512U
#define COMMAND_LIMIT      127U
/* 以下时间单位都是毫秒：有效速度命令需持续刷新，遥测每秒发送约10次。 */
#define COMMAND_TIMEOUT_MS 300U
#define ARM_QUIET_MS       250U
#define TELEMETRY_MS       100U
/* 遥控模式的独立轮速目标上限；不同于 motor4.h 中底层允许的最大轮速。 */
#define REMOTE_MAX_RPM     60.0f

/* 单生产者（USART2 IRQ）/单消费者（主循环）缓冲；时间戳防止积压旧帧
 * 在主循环恢复后被当成刚收到的新运动指令。所有共享状态均用短临界区保护。 */
static uint8_t s_rx_byte;
static volatile uint8_t s_rx[RX_CAPACITY];
static volatile uint32_t s_rx_at[RX_CAPACITY];
/* head 是中断下次写入的位置，tail 是主循环下次读取的位置。
 * 每个字节都附带真正到达时刻，组帧超时不从主循环取出字节时重新计算。 */
static volatile uint16_t s_head, s_tail;
static volatile bool s_drop_rx;
static volatile bool s_ready;
static volatile bool s_armed;
/* session 区分不同上位机连接；last_seq 区分同一连接中的命令先后；
 * valid_at 记录最近一次使能/有效速度命令的时间，查询和坏帧不续期。 */
static volatile uint32_t s_session, s_last_seq, s_valid_at;
/* generation 是本地撤销许可的代次。停机即加一，令已取出的旧半帧也失效。 */
static volatile uint32_t s_generation;
static uint32_t s_line_generation;
static char s_line[COMMAND_LIMIT + 1U];
static size_t s_line_length;
static uint32_t s_line_at;
static uint32_t s_last_motion_ms, s_print_ms;
static int64_t s_position[MOTOR4_COUNT];

/* 与 motor4 的临界区规则一致：保存后恢复原PRIMASK，可从主循环或中断嵌套调用。 */
static uint32_t Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static void Unlock(uint32_t key) { __set_PRIMASK(key); }

/* USER 按键低电平有效；遥控模式中它只用于本地撤销运动许可。 */
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
    /* 四轮故障按位合并；网络端只需检查非零，就知道整车已有故障。 */
    uint8_t faults = 0U;
    for (unsigned i = 0; i < MOTOR4_COUNT; ++i) faults |= t[i].fault;
    return faults;
}

/* CRC16-CCITT-FALSE：初值0xFFFF、多项式0x1021，与 host_control/robot_link/protocol.py 一致。
 * 只计算正文，不把 @、*、校验数字和换行计入。它用于检测传输内容是否被破坏。 */
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
    /* 未使能时坏帧不触发运动；已使能时立即撤销许可，等待重新显式使能。 */
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
    /* ACK 表示一条命令的处理结果；带回原会话和序号，供 Mac 对应正在等待的请求。
     * ACK 的 OK 与实际轮速已达到目标是两件事，轮速另见 T 遥测帧。 */
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
    /* 速度允许负号；先读受限的无符号幅值，再加符号，避免越界整数参与控制。
     * vx/vy 的 limit=150（mm/s），wz 的 limit=600（mrad/s）。 */
    bool negative = *text == '-';
    if (negative) ++text;
    uint32_t magnitude;
    if (!ParseU32(text, &magnitude) || magnitude > (uint32_t)limit) return false;
    *out = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return true;
}

static bool NewSequence(uint32_t seq)
{
    /* 无符号模差允许序号从UINT32_MAX回到0；差为0是重播，差跨半个范围视为旧序号。
     * 只有“比上一条新”的命令才有机会改变当前会话的状态。 */
    uint32_t distance = seq - s_last_seq;
    return distance != 0U && distance < 0x80000000U;
}

/* 一次校验四轮估算峰值，再按同一比例降低整个车身指令，保持方向/曲率。
 * 限制的是轮速目标，不是硬件电流；60RPM 是此阶段遥控的调速上限。 */
static bool SetBodyVelocity(int32_t vx, int32_t vy, int32_t wz)
{
    /* 尺寸宏原单位为mm：直径/完整轮距要同时“除2取半值、除1000换成米”。
     * 收到的平移mm/s和旋转mrad/s则只需除1000，得到m/s与rad/s。 */
    const float radius = ROBOT_APP_WHEEL_DIAMETER_MM / 2000.0f;
    const float half_l = ROBOT_APP_WHEELBASE_MM / 2000.0f;
    const float half_w = ROBOT_APP_TRACK_MM / 2000.0f;
    float f = (float)vx / 1000.0f;
    float l = (float)vy / 1000.0f;
    float t = (float)wz / 1000.0f;
    const float rpm_factor = 60.0f / (2.0f * 3.14159265358979323846f * radius);
    float rotation = (half_l + half_w) * t;
    /* 此时四轮数组是轮缘线速度（m/s），再乘 rpm_factor 才是输出轴RPM。
     * 在交给 mecanum 前再做遥控阶段的60RPM上限，四轮始终等比例缩放。 */
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

/* CRC和帧边界已通过后才进入命令解释；payload 可修改，逗号会原地改成字符串结尾。
 * H=session握手，A=使能，S=停止，V=车身速度；每种命令字段数固定。 */
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
        /* 新连接从 H,<非零session>,0 开始。握手先停机，再替换会话身份；
         * 不会因为握手成功就自动允许V命令驱动车轮。 */
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
            /* 当前会话的新STOP可以在未使能或已有故障时执行；故障本身仍保留。 */
            DisarmLocked();
        } else if (FaultMask(t) != 0U) {
            DisarmLocked();
            result = "FAULT";
        } else if (type == 'A') {
            /* ARM要求四轮已处于STOP、最近至少250ms未观察到运动、USER未按下。
             * 它只打开接收运动目标的许可，真正目标必须由后续V命令给出。 */
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
            /* 唯一剩余类型为V：三个轴依次为前进、左移、俯视逆时针角速度。
             * 解析、许可、时效、四轮命令接受条件必须全部通过，才刷新valid_at。 */
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

/* s_line已收齐一行且末尾有\0；先验证 @正文*XXXX 的结构和CRC，
 * 再把正文交给Execute。换行符本身已由Poll消费，不在s_line中。 */
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
    /* 15.0RPM -> 150；使用整数传输可省去嵌入式浮点printf支持。
     * 超大或非有限数先处理，再转换为long，避免未定义的浮点转整数结果。 */
    double value = (double)rpm * 10.0;
    if (!isfinite(value)) return 0;
    if (value > (double)INT32_MAX) return (long)INT32_MAX;
    if (value < (double)INT32_MIN) return (long)INT32_MIN;
    return (long)value;
}

static void PrintTelemetry(void)
{
    /* 先在短临界区内复制一次一致快照，再在开中断状态下格式化和发送。
     * 正文顺序为：T/session/毫秒时刻/使能/最近序号/故障，随后四组四轮字段：
     * count[4]、实际RPM×10[4]、目标RPM×10[4]、逻辑PWM[4]。
     * 各组轮序均为 FL、FR、RL、RR；Mac 的 parse_serial 按完全相同顺序解包。 */
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
    /* 在 Motor4_Init 之后调用一次；清空旧会话和残留数据，并明确停止全部轮子。
     * s_print_ms回退一个遥测周期，使第一次Poll便有机会发出初始状态。 */
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
    /* USART2每收满1字节，HAL回调转到这里。只入队，不在中断里计算CRC、解析或打印。
     * 写head之前先保存字节和时间，消费者才能读到完整的一对数据。 */
    if (uart != &huart2 || !s_ready) return;
    uint32_t key = Lock();
    uint8_t byte = s_rx_byte;
    uint16_t next = (uint16_t)((s_head + 1U) % RX_CAPACITY);
    if (s_drop_rx) {
        if (byte == '\n') s_drop_rx = false;
    } else if (next == s_tail) {
        /* 队列满意味着数据无法完整保留：立即停机，并丢到本行换行处再恢复收集。 */
        DisarmLocked();
        s_drop_rx = byte != '\n';
    } else {
        s_rx[s_head] = byte;
        s_rx_at[s_head] = HAL_GetTick();
        s_head = next;
    }
    Unlock(key);
    /* HAL单字节中断接收完成后不会自动等待下一字节，必须重新挂起接收。 */
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        RemoteControl_UART_Error(uart);
    }
}

void RemoteControl_UART_Error(UART_HandleTypeDef *uart)
{
    /* 接收出错先取消运行许可和旧数据；重新挂起接收只恢复通信，不恢复运动。 */
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
    /* 与四轮PI共用TIM9节拍，因此主循环即使暂时阻塞，也能检查按钮与300ms时效。
     * 读取位置还用于下一次ARM的停稳条件；此函数不会解析收到的命令。 */
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
    /* 主循环中的通信状态机：消费字节 -> 收齐一行 -> 校验并执行 -> 定期回传遥测。
     * 每次Poll可处理零个或多个完整帧，半帧保留到下一次Poll。 */
    if (!s_ready) return;
    /* 有界地消费一批字节，防止持续输入霸占主循环。 */
    for (unsigned processed = 0U; processed < RX_CAPACITY; ++processed) {
        uint32_t key = Lock();
        if (s_line_generation != s_generation) {
            /* ISR可能在主循环两次取字节之间停机；代次不符时废弃已拼接的旧前缀。 */
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
            /* 只允许在空行状态开始新帧；旧半帧中嵌入第二个@视为格式错误。 */
            if (s_line_length != 0U) { BadFrame(); continue; }
            s_line_at = arrived;
            s_line[s_line_length++] = '@';
        } else if (byte == '\n') {
            /* 支持LF或CRLF结尾；以首字节到达时刻判断整帧是否过期。 */
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
        /* 遥控300ms许可和电机500ms看门狗分层：主循环确认许可仍有效，才喂电机狗。
         * Tick只负责撤销，不能替主循环持续喂狗，否则会掩盖主循环停滞。 */
        Motor4_FeedWatchdog();
    }
    bool print = (uint32_t)(now - s_print_ms) >= TELEMETRY_MS;
    if (print) s_print_ms = now;
    Unlock(key);
    if (print) PrintTelemetry();
}
