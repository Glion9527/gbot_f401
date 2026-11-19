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
    /* 四轮累计计数与转速，供 ROS 2 /joint_states 使用。 */
    struct WheelState {
        std::array<std::int64_t, LINK_V2_WHEEL_COUNT> counts{};
        std::array<double, LINK_V2_WHEEL_COUNT> rpm{};
    };

    /* 最近一次 TELEM 携带的 IMU 原始整数量。 */
    struct ImuState {
        std::uint16_t flags = 0;
        std::array<std::int16_t, 3> acc_mg{};
        std::array<std::int16_t, 3> gyro_dps10{};
        std::array<std::int16_t, 3> mag_ut10{};
        std::int16_t temp_c10 = 0;
    };

    explicit SerialBase(const std::string& device, int baud_rate = 115200);

    bool setVelocity(const core::ChassisSpeeds& cmd) override;
    std::optional<hal::OdometryState> getOdometry() const override;
    void poll() override;

    /* 显式使能/解除使能。每帧 CMD 都会携带当前状态。 */
    void setArmed(bool armed) noexcept { armed_ = armed; }
    bool armed() const noexcept { return armed_; }

    /* 累计收到的 TELEM 帧数，用于验收 50Hz 和排查丢帧。 */
    std::uint32_t telemetryCount() const noexcept { return telemetry_count_; }

    /* 最近一次 TELEM 的四轮累计计数/转速；遥测过期返回 nullopt。 */
    std::optional<WheelState> getWheelState() const;

    /* 最近一次 TELEM 的 IMU 原始值；遥测过期返回 nullopt。 */
    std::optional<ImuState> getImu() const;

    /* 新进程启动时把本地 seq 对齐到 STM32 的 last_seq。
     * v2 没有会话握手，STM32 会跨 host 进程保留 s_last_seq；如果 host 总从 1 开始，
     * 上一轮跑过之后的新序号会被当成旧帧丢弃，ARM/速度永远被拒绝。 */
    bool syncSequence(std::chrono::milliseconds timeout);

    /* 最近一次 I/O 失败原因；空字符串表示没有错误。 */
    const std::string& lastError() const noexcept { return last_error_; }

private:
    void applyTelemetry(const Telemetry& telemetry);

    SerialPort port_;
    LinkV2Decoder decoder_;
    std::uint16_t seq_ = 0;
    bool armed_ = false;
    std::uint32_t telemetry_count_ = 0;
    std::array<std::int64_t, LINK_V2_WHEEL_COUNT> wheel_counts_{};
    std::array<double, LINK_V2_WHEEL_COUNT> wheel_rpm_{};
    ImuState imu_{};
    hal::OdometryState odometry_{};
    std::optional<std::chrono::steady_clock::time_point> last_telemetry_;
    std::string last_error_;
    std::array<std::uint8_t, 256> read_buffer_{};
};

}  // namespace f401::drivers
