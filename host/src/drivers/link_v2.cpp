#include "f401/drivers/link_v2.hpp"

#include <algorithm>
#include <cstddef>

namespace f401::drivers {
namespace {

/* 显式小端读写。不要对整个结构体做 memcpy：
 * 结构体可能有 padding，且不同编译器/架构的对齐规则不一致。 */
void putU16(std::uint8_t* out, std::uint16_t value)
{
    out[0] = static_cast<std::uint8_t>(value & 0xFFu);
    out[1] = static_cast<std::uint8_t>((value >> 8) & 0xFFu);
}

void putI16(std::uint8_t* out, std::int16_t value)
{
    putU16(out, static_cast<std::uint16_t>(value));
}

std::uint16_t getU16(const std::uint8_t* in)
{
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[0]) |
                                      (static_cast<std::uint16_t>(in[1]) << 8));
}

std::int16_t getI16(const std::uint8_t* in)
{
    return static_cast<std::int16_t>(getU16(in));
}

}  // namespace

std::array<std::uint8_t, LINK_V2_CMD_SIZE> encodeCommand(const WheelCommand& command)
{
    std::array<std::uint8_t, LINK_V2_CMD_SIZE> frame{};
    frame[LINK_V2_CMD_OFF_HEADER0] = static_cast<std::uint8_t>(LINK_V2_HEADER0);
    frame[LINK_V2_CMD_OFF_HEADER1] = static_cast<std::uint8_t>(LINK_V2_HEADER1);
    frame[LINK_V2_CMD_OFF_VERSION] = static_cast<std::uint8_t>(LINK_V2_VERSION);
    frame[LINK_V2_CMD_OFF_TYPE] = static_cast<std::uint8_t>(LINK_V2_TYPE_CMD);
    putU16(&frame[LINK_V2_CMD_OFF_SEQ], command.seq);
    putU16(&frame[LINK_V2_CMD_OFF_FLAGS], command.flags);
    for (std::size_t index = 0; index < LINK_V2_WHEEL_COUNT; ++index) {
        putI16(&frame[LINK_V2_CMD_OFF_WHEELS + index * 2u], command.wheel_mm_s[index]);
    }

    /* CRC 覆盖 version..payload，不覆盖帧头和 CRC 自身，和文档表格一致。 */
    const std::uint16_t crc =
        link_v2_crc16(&frame[LINK_V2_CMD_OFF_VERSION],
                      LINK_V2_CMD_OFF_CRC - LINK_V2_CMD_OFF_VERSION);
    putU16(&frame[LINK_V2_CMD_OFF_CRC], crc);
    return frame;
}

std::optional<WheelCommand> decodeCommand(std::span<const std::uint8_t> frame)
{
    if (frame.size() != LINK_V2_CMD_SIZE) {
        return std::nullopt;
    }
    if (frame[LINK_V2_CMD_OFF_HEADER0] != LINK_V2_HEADER0 ||
        frame[LINK_V2_CMD_OFF_HEADER1] != LINK_V2_HEADER1 ||
        frame[LINK_V2_CMD_OFF_VERSION] != LINK_V2_VERSION ||
        frame[LINK_V2_CMD_OFF_TYPE] != LINK_V2_TYPE_CMD) {
        return std::nullopt;
    }

    const std::uint16_t expected = getU16(&frame[LINK_V2_CMD_OFF_CRC]);
    const std::uint16_t actual =
        link_v2_crc16(&frame[LINK_V2_CMD_OFF_VERSION],
                      LINK_V2_CMD_OFF_CRC - LINK_V2_CMD_OFF_VERSION);
    if (expected != actual) {
        return std::nullopt;
    }

    WheelCommand command{};
    command.seq = getU16(&frame[LINK_V2_CMD_OFF_SEQ]);
    command.flags = getU16(&frame[LINK_V2_CMD_OFF_FLAGS]);
    for (std::size_t index = 0; index < LINK_V2_WHEEL_COUNT; ++index) {
        command.wheel_mm_s[index] =
            getI16(&frame[LINK_V2_CMD_OFF_WHEELS + index * 2u]);
    }
    return command;
}

std::optional<Telemetry> LinkV2Decoder::feed(std::span<const std::uint8_t> bytes)
{
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());

    /* 防御性上限：正常流不会超过几帧；异常主机持续灌垃圾时避免无限增长。 */
    if (buffer_.size() > 4096u) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 2048);
    }

    for (;;) {
        if (buffer_.size() < 2u) {
            return std::nullopt;
        }

        /* 找帧头：不在帧头就从缓冲区头部丢弃，直到第一个 0xA5。 */
        if (buffer_[0] != LINK_V2_HEADER0) {
            const auto header =
                std::find(buffer_.begin(), buffer_.end(), static_cast<std::uint8_t>(LINK_V2_HEADER0));
            buffer_.erase(buffer_.begin(), header);
            continue;
        }
        if (buffer_[1] != LINK_V2_HEADER1) {
            buffer_.erase(buffer_.begin());
            continue;
        }
        if (buffer_.size() < LINK_V2_TELEM_SIZE) {
            return std::nullopt; /* 帧还没收全，等下一次 feed */
        }

        const std::uint8_t* frame = buffer_.data();
        const bool header_ok =
            frame[LINK_V2_TELEM_OFF_VERSION] == LINK_V2_VERSION &&
            frame[LINK_V2_TELEM_OFF_TYPE] == LINK_V2_TYPE_TELEM;
        const std::uint16_t expected = getU16(&frame[LINK_V2_TELEM_OFF_CRC]);
        const std::uint16_t actual =
            link_v2_crc16(&frame[LINK_V2_TELEM_OFF_VERSION],
                          LINK_V2_TELEM_OFF_CRC - LINK_V2_TELEM_OFF_VERSION);
        if (!header_ok || expected != actual) {
            /* 坏帧：只丢一字节，保留帧内可能存在的下一个帧头，保证能重新同步。 */
            buffer_.erase(buffer_.begin());
            continue;
        }

        Telemetry telemetry{};
        telemetry.seq = getU16(&frame[LINK_V2_TELEM_OFF_SEQ]);
        telemetry.mcu_ms = getU16(&frame[LINK_V2_TELEM_OFF_MCU_MS]);
        telemetry.status = getU16(&frame[LINK_V2_TELEM_OFF_STATUS]);
        telemetry.fault_mask = getU16(&frame[LINK_V2_TELEM_OFF_FAULT]);
        for (std::size_t index = 0; index < LINK_V2_WHEEL_COUNT; ++index) {
            telemetry.delta[index] =
                getI16(&frame[LINK_V2_TELEM_OFF_DELTA + index * 2u]);
            telemetry.rpm_x10[index] =
                getI16(&frame[LINK_V2_TELEM_OFF_RPM10 + index * 2u]);
        }

        buffer_.erase(buffer_.begin(), buffer_.begin() + LINK_V2_TELEM_SIZE);
        return telemetry;
    }
}

}  // namespace f401::drivers
