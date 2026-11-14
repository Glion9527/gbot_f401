#pragma once

/*
 * IBase 的串口实现：把 ChassisSpeeds 转成四轮速度，再用 Link v2 发给 STM32；
 * 同时把收到的 TELEM 帧积分成里程计。
 *
 * 当前是单线程骨架：主循环调用 poll() 处理串口输入，
 * setVelocity()/getOdometry() 不做跨线程同步。线程化留给阶段 2。
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "f401/drivers/link_v2.hpp"
#include "f401/drivers/serial_port.hpp"
#include "f401/hal/i_base.hpp"

namespace f401::drivers {

class SerialBase final : public hal::IBase {
public:
    explicit SerialBase(const std::string& device, int baud_rate = 115200);

    bool setVelocity(const core::ChassisSpeeds& cmd) override;
    std::optional<hal::OdometryState> getOdometry() const override;
    void poll() override;

    /* 显式使能/解除使能。每帧 CMD 都会携带当前状态。 */
    void setArmed(bool armed) noexcept { armed_ = armed; }
    bool armed() const noexcept { return armed_; }

    /* 最近一次 I/O 失败原因；空字符串表示没有错误。 */
    const std::string& lastError() const noexcept { return last_error_; }

private:
    void applyTelemetry(const Telemetry& telemetry);

    SerialPort port_;
    LinkV2Decoder decoder_;
    std::uint16_t seq_ = 0;
    bool armed_ = false;
    hal::OdometryState odometry_{};
    std::optional<std::chrono::steady_clock::time_point> last_telemetry_;
    std::string last_error_;
    std::array<std::uint8_t, 256> read_buffer_{};
};

}  // namespace f401::drivers
