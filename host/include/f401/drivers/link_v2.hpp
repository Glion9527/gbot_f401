#pragma once

/*
 * Link v2 二进制帧编解码（传输/协议层）。
 *
 * 只负责“字节 <-> 结构体”，不碰串口、不碰 ROS。
 * 所有多字节整数显式小端序，绝不直接 memcpy 结构体（避免 padding/对齐差异）。
 *
 * 解码器是流式的：可以一次喂一个字节，也可以喂一大块；
 * 遇到垃圾字节、坏 CRC、错误版本会自动丢一字节重新找帧头，
 * 这样拔插线、复位后残留的半帧不会让状态机卡死。
 */

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "link_v2.h"

namespace f401::drivers {

struct WheelCommand {
    std::uint16_t seq = 0;
    std::uint16_t flags = 0;
    std::array<std::int16_t, LINK_V2_WHEEL_COUNT> wheel_mm_s{};
};

struct Telemetry {
    std::uint16_t seq = 0;
    std::uint16_t mcu_ms = 0;
    std::uint16_t status = 0;
    std::uint16_t fault_mask = 0;
    std::array<std::int16_t, LINK_V2_WHEEL_COUNT> delta{};   // 编码器增量
    std::array<std::int16_t, LINK_V2_WHEEL_COUNT> rpm_x10{}; // 转速 ×10
};

/* 编码一帧 CMD（18 字节），CRC 覆盖 version..wheels。 */
std::array<std::uint8_t, LINK_V2_CMD_SIZE> encodeCommand(const WheelCommand& command);

/* 解析一帧完整 CMD；长度/CRC/版本/类型任一不符返回 nullopt。
 * 主要给单元测试和将来的固件对照用，真实串口方向只发不收 CMD。 */
std::optional<WheelCommand> decodeCommand(std::span<const std::uint8_t> frame);

/* 流式 TELEM 解码器。feed() 返回解析出的一帧后就返回；
 * 如果缓冲区里还有完整帧，下一次 feed({}) 会继续解析。 */
class LinkV2Decoder {
public:
    std::optional<Telemetry> feed(std::span<const std::uint8_t> bytes);
    void reset() { buffer_.clear(); }

private:
    std::vector<std::uint8_t> buffer_;
};

}  // namespace f401::drivers
