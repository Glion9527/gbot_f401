/* Link v2 编解码测试：黄金帧、CRC、流式解码、坏帧重同步。 */

#include <array>
#include <cstdint>
#include <span>

#include "f401/drivers/link_v2.hpp"
#include "test_support.hpp"

namespace {

using f401::drivers::LinkV2Decoder;
using f401::drivers::Telemetry;
using f401::drivers::WheelCommand;

/* 与 protocol/tests/golden_frames.txt 完全一致的字节。 */
constexpr std::array<std::uint8_t, LINK_V2_CMD_SIZE> kCmdArmZero = {
    0xA5, 0x5A, 0x02, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFB, 0x76};

constexpr std::array<std::uint8_t, LINK_V2_CMD_SIZE> kCmdDrive300 = {
    0xA5, 0x5A, 0x02, 0x01, 0x02, 0x00, 0x01, 0x00, 0x2C,
    0x01, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x86, 0x29};

constexpr std::array<std::uint8_t, LINK_V2_TELEM_SIZE> kTelemetry = {
    0xA5, 0x5A, 0x02, 0x81, 0x02, 0x00, 0xD2, 0x04, 0x01, 0x00,
    0x00, 0x00, 0x0A, 0x00, 0xEC, 0xFF, 0x1E, 0x00, 0xD8, 0xFF,
    0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x36, 0xDC};

void testCrcVector()
{
    const char* text = "123456789";
    const auto crc = link_v2_crc16(reinterpret_cast<const std::uint8_t*>(text), 9);
    CHECK_EQ(crc, std::uint16_t{0x29B1});
}

void testEncodeGolden()
{
    WheelCommand command{};
    command.seq = 1;
    command.flags = LINK_V2_FLAG_ARM;
    command.wheel_mm_s = {0, 0, 0, 0};
    CHECK(f401::drivers::encodeCommand(command) == kCmdArmZero);

    command.seq = 2;
    command.wheel_mm_s = {300, 300, 300, 300};
    CHECK(f401::drivers::encodeCommand(command) == kCmdDrive300);
}

void testDecodeCommand()
{
    const auto command = f401::drivers::decodeCommand(kCmdDrive300);
    CHECK(command.has_value());
    if (command) {
        CHECK_EQ(command->seq, std::uint16_t{2});
        CHECK_EQ(command->flags, LINK_V2_FLAG_ARM);
        CHECK_EQ(command->wheel_mm_s[0], std::int16_t{300});
        CHECK_EQ(command->wheel_mm_s[3], std::int16_t{300});
    }

    auto broken = kCmdDrive300;
    broken.back() ^= 0xFF;
    CHECK(!f401::drivers::decodeCommand(broken).has_value());

    auto wrong_version = kCmdDrive300;
    wrong_version[LINK_V2_CMD_OFF_VERSION] = 0x09;
    CHECK(!f401::drivers::decodeCommand(wrong_version).has_value());
}

void testStreamingTelemetry()
{
    LinkV2Decoder decoder;
    const auto telemetry = decoder.feed(kTelemetry);
    CHECK(telemetry.has_value());
    if (telemetry) {
        CHECK_EQ(telemetry->seq, std::uint16_t{2});
        CHECK_EQ(telemetry->mcu_ms, std::uint16_t{1234});
        CHECK_EQ(telemetry->status, LINK_V2_STATUS_ARMED);
        CHECK_EQ(telemetry->fault_mask, std::uint16_t{0});
        CHECK_EQ(telemetry->delta[0], std::int16_t{10});
        CHECK_EQ(telemetry->delta[1], std::int16_t{-20});
        CHECK_EQ(telemetry->delta[2], std::int16_t{30});
        CHECK_EQ(telemetry->delta[3], std::int16_t{-40});
        CHECK_EQ(telemetry->rpm_x10[0], std::int16_t{300});
    }
}

void testGarbageResync()
{
    LinkV2Decoder decoder;
    const std::array<std::uint8_t, 3> garbage = {0x00, 0x11, 0x22};
    CHECK(!decoder.feed(garbage).has_value());
    const auto telemetry = decoder.feed(kTelemetry);
    CHECK(telemetry.has_value());
}

void testSplitAndCorrupt()
{
    LinkV2Decoder decoder;
    const auto first = std::span<const std::uint8_t>(kTelemetry.data(), 7);
    CHECK(!decoder.feed(first).has_value());
    const auto rest = std::span<const std::uint8_t>(kTelemetry.data() + 7, kTelemetry.size() - 7);
    CHECK(decoder.feed(rest).has_value());

    LinkV2Decoder bytes;
    std::optional<Telemetry> last;
    for (std::size_t index = 0; index < kTelemetry.size(); ++index) {
        last = bytes.feed(std::span<const std::uint8_t>(kTelemetry.data() + index, 1));
    }
    CHECK(last.has_value());

    LinkV2Decoder corrupt;
    auto broken = kTelemetry;
    broken.back() ^= 0xFF;
    CHECK(!corrupt.feed(broken).has_value());

    auto wrong_version = kTelemetry;
    wrong_version[LINK_V2_TELEM_OFF_VERSION] = 0x09;
    CHECK(!decoder.feed(wrong_version).has_value());
}

void testBackToBackFrames()
{
    std::array<std::uint8_t, LINK_V2_TELEM_SIZE * 2> two{};
    for (std::size_t index = 0; index < LINK_V2_TELEM_SIZE; ++index) {
        two[index] = kTelemetry[index];
        two[index + LINK_V2_TELEM_SIZE] = kTelemetry[index];
    }
    LinkV2Decoder decoder;
    CHECK(decoder.feed(two).has_value());
    CHECK(decoder.feed({}).has_value());
}

}  // namespace

int main()
{
    testCrcVector();
    testEncodeGolden();
    testDecodeCommand();
    testStreamingTelemetry();
    testGarbageResync();
    testSplitAndCorrupt();
    testBackToBackFrames();
    return test::finish("link_v2 codec");
}
