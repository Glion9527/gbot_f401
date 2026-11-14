#ifndef F401_PROTOCOL_LINK_V2_H
#define F401_PROTOCOL_LINK_V2_H

/*
 * STM32 <-> 树莓派 二进制链路协议 v2 的唯一契约。
 *
 * 设计原则：
 *  - 固定长度帧，避免 ASCII 字符串解析和浮点跨平台问题；
 *  - 多字节整数一律小端（LE），不要直接 memcpy 结构体（会有 padding）；
 *  - CRC16-CCITT-FALSE，初值 0xFFFF、多项式 0x1021，和现有
 *    Core/Src/remote_control.c 与 host_control/robot_link/protocol.py 一致；
 *  - 速度单位 mm/s，带符号：FL/FR/RL/RR 逻辑正方向都表示“推动小车前进”；
 *  - 命令帧带 seq 和 ARM/DISARM 标志，保留现有“显式使能 + 超时停机”的安全模型。
 *
 * 该头文件同时被固件 C 代码和宿主 C++ 代码 include，是两边唯一的契约来源。
 * 修改字段时：先改这里和 protocol/tests/golden_frames.txt，再改两端实现，
 * 最后跑 host 的 codec 测试和固件侧的测试。
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 帧头：固定两个字节，便于从垃圾字节中重新同步。 */
#define LINK_V2_HEADER0 0xA5u
#define LINK_V2_HEADER1 0x5Au

/* 协议版本和帧类型。version 用于未来升级；type 区分方向/用途。 */
#define LINK_V2_VERSION 0x02u
#define LINK_V2_TYPE_CMD 0x01u   /* 树莓派 -> STM32 */
#define LINK_V2_TYPE_TELEM 0x81u /* STM32 -> 树莓派 */

/* 帧长：固定长度，接收方按长度取帧，CRC 校验失败则丢一字节重新同步。 */
#define LINK_V2_CMD_SIZE 18u
#define LINK_V2_TELEM_SIZE 30u
#define LINK_V2_WHEEL_COUNT 4u

/* CMD 帧偏移（18 字节）。 */
#define LINK_V2_CMD_OFF_HEADER0 0u
#define LINK_V2_CMD_OFF_HEADER1 1u
#define LINK_V2_CMD_OFF_VERSION 2u
#define LINK_V2_CMD_OFF_TYPE 3u
#define LINK_V2_CMD_OFF_SEQ 4u
#define LINK_V2_CMD_OFF_FLAGS 6u
#define LINK_V2_CMD_OFF_WHEELS 8u  /* 4 × int16：FL, FR, RL, RR */
#define LINK_V2_CMD_OFF_CRC 16u

/* TELEM 帧偏移（30 字节）。 */
#define LINK_V2_TELEM_OFF_HEADER0 0u
#define LINK_V2_TELEM_OFF_HEADER1 1u
#define LINK_V2_TELEM_OFF_VERSION 2u
#define LINK_V2_TELEM_OFF_TYPE 3u
#define LINK_V2_TELEM_OFF_SEQ 4u
#define LINK_V2_TELEM_OFF_MCU_MS 6u
#define LINK_V2_TELEM_OFF_STATUS 8u
#define LINK_V2_TELEM_OFF_FAULT 10u
#define LINK_V2_TELEM_OFF_DELTA 12u /* 4 × int16：自上一 TELEM 以来的编码器增量 */
#define LINK_V2_TELEM_OFF_RPM10 20u /* 4 × int16：转速 ×10 */
#define LINK_V2_TELEM_OFF_CRC 28u

/* CMD flags：bit0 使能，bit1 解除使能，bit2 急停。
 * 每帧都携带当前意图；STM32 只在“显式 ARM 且满足停稳/无故障条件”时接受运动。 */
#define LINK_V2_FLAG_ARM 0x0001u
#define LINK_V2_FLAG_DISARM 0x0002u
#define LINK_V2_FLAG_ESTOP 0x0004u

/* TELEM status 位。 */
#define LINK_V2_STATUS_ARMED 0x0001u
#define LINK_V2_STATUS_FAULT 0x0002u
#define LINK_V2_STATUS_ESTOP 0x0004u
#define LINK_V2_STATUS_CMD_TIMEOUT 0x0008u

/* 时间参数：沿用现有固件的 300ms 遥控超时；遥测 50Hz。 */
#define LINK_V2_CMD_TIMEOUT_MS 300u
#define LINK_V2_TELEM_PERIOD_MS 20u

/* CRC16-CCITT-FALSE：和现有 ASCII v1 协议使用同一个多项式。
 * 标准测试向量："123456789" -> 0x29B1。 */
static inline uint16_t link_v2_crc16(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFFu;
    size_t i;
    unsigned bit;
    for (i = 0u; i < size; ++i) {
        crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)data[i] << 8));
        for (bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* 编译期校验帧长和字段偏移，防止两端“看起来一致、实际错位”。 */
#if defined(__cplusplus)
static_assert(LINK_V2_CMD_SIZE == 18u, "CMD frame must stay 18 bytes");
static_assert(LINK_V2_TELEM_SIZE == 30u, "TELEM frame must stay 30 bytes");
static_assert(LINK_V2_CMD_OFF_CRC + 2u == LINK_V2_CMD_SIZE, "CMD CRC must be last");
static_assert(LINK_V2_TELEM_OFF_CRC + 2u == LINK_V2_TELEM_SIZE, "TELEM CRC must be last");
#else
_Static_assert(LINK_V2_CMD_SIZE == 18u, "CMD frame must stay 18 bytes");
_Static_assert(LINK_V2_TELEM_SIZE == 30u, "TELEM frame must stay 30 bytes");
_Static_assert(LINK_V2_CMD_OFF_CRC + 2u == LINK_V2_CMD_SIZE, "CMD CRC must be last");
_Static_assert(LINK_V2_TELEM_OFF_CRC + 2u == LINK_V2_TELEM_SIZE, "TELEM CRC must be last");
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* F401_PROTOCOL_LINK_V2_H */
