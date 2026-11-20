#include "f401/drivers/serial_base.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>

#include "robot_geometry.h"

namespace f401::drivers {
namespace {

/* M5 实测校准（2026-11-19，3 组）：
 *   实际 1.000m -> odom 0.964/0.977/0.965m   -> k≈1.0324
 *   实际 360°  -> odom 349.02/349.80/347.21° -> k≈1.0325
 * 前进和旋转的误差比例一致，说明主要是轮缘位移的尺度偏小；
 * 用一个统一轮速比例同时修正距离和由轮速反算的 yaw。
 * 横移 0.5m 实测 0.499/0.491/0.488m，单独 k≈1.015；套用该统一比例后
 * 约 0.509m，误差 1.7%，仍在 M5 的 5% 目标内。 */
constexpr double kMetersPerCount =
    f401::core::kPi * (F401_WHEEL_DIAMETER_MM / 1000.0) / F401_ENCODER_CPR *
    SerialBase::kOdomWheelScale;

/* TELEM 超过这个时间没更新，就认为里程计失效，不再返回旧位姿。 */
constexpr auto kTelemetryTimeout = std::chrono::milliseconds(300);

}  // namespace

SerialBase::SerialBase(const std::string& device, int baud_rate)
    : port_(device, baud_rate)
{
}

bool SerialBase::setVelocity(const core::ChassisSpeeds& cmd)
{
    /* 先做轴限幅，再把车身速度分解成四轮；最后按 75RPM 对应的轮缘线速度
     * 做等比例限幅，和固件 v1 的行为保持一致（v2 固件仍会在 MCU 侧再兜底）。 */
    const core::ChassisSpeeds limited = core::clampChassisSpeeds(cmd);
    core::WheelSpeeds wheel = core::toWheelSpeeds(limited);
    wheel = core::limitWheelSpeeds(wheel, core::kRemoteWheelLinearLimitMps);

    WheelCommand frame{};
    frame.seq = ++seq_;
    frame.flags = armed_ ? LINK_V2_FLAG_ARM : LINK_V2_FLAG_DISARM;

    /* 未使能时强制零速，避免“上层算了半天，电机却收到旧目标”。 */
    if (!armed_) {
        wheel = core::WheelSpeeds{};
    }

    const double speeds[LINK_V2_WHEEL_COUNT] = {wheel.fl_mps, wheel.fr_mps, wheel.rl_mps,
                                                wheel.rr_mps};
    for (std::size_t index = 0; index < LINK_V2_WHEEL_COUNT; ++index) {
        long value = std::lround(speeds[index] * 1000.0);
        value = std::clamp<long>(value, std::numeric_limits<std::int16_t>::min(),
                                 std::numeric_limits<std::int16_t>::max());
        frame.wheel_mm_s[index] = static_cast<std::int16_t>(value);
    }

    try {
        const auto bytes = encodeCommand(frame);
        port_.writeAll(bytes, std::chrono::milliseconds(50));
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        last_error_ = error.what();
        return false;
    }
}

bool SerialBase::syncSequence(std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        poll();
        if (!last_error_.empty()) {
            return false;
        }
        if (last_telemetry_) {
            /* seq_ 是“上一帧已用序号”；setVelocity 会先 ++seq_，
             * 所以这里直接取 TELEM 回显的 last_seq，下一帧就是 last_seq+1。 */
            seq_ = odometry_.seq;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

void SerialBase::poll()
{
    /* 非阻塞地尽量多读：每次 readSome(0ms) 有多少读多少，
     * 然后交给流式解码器；一帧解析完继续看缓冲区里是否还有下一帧。 */
    for (;;) {
        std::size_t count = 0;
        try {
            count = port_.readSome(read_buffer_, std::chrono::milliseconds(0));
        } catch (const std::exception& error) {
            last_error_ = error.what();
            return;
        }
        if (count == 0) {
            return;
        }

        std::optional<Telemetry> telemetry =
            decoder_.feed(std::span<const std::uint8_t>(read_buffer_.data(), count));
        while (telemetry) {
            applyTelemetry(*telemetry);
            telemetry = decoder_.feed({});
        }
    }
}

void SerialBase::applyTelemetry(const Telemetry& telemetry)
{
    const auto now = std::chrono::steady_clock::now();
    ++telemetry_count_;

    for (std::size_t index = 0; index < LINK_V2_WHEEL_COUNT; ++index) {
        wheel_counts_[index] += telemetry.delta[index];
        wheel_rpm_[index] = telemetry.rpm_x10[index] / 10.0;
    }
    imu_.flags = telemetry.imu_flags;
    imu_.acc_mg = telemetry.acc_mg;
    imu_.gyro_dps10 = telemetry.gyro_dps10;
    imu_.mag_ut10 = telemetry.mag_ut10;
    imu_.temp_c10 = telemetry.temp_c10;

    /* 第一帧没有 dt，只记录时间；之后用实际到达间隔积分。
     * 间隔过大（例如拔线重连）说明中间丢了帧，跳过本次积分避免位姿跳变。 */
    if (last_telemetry_) {
        const double dt = std::chrono::duration<double>(now - *last_telemetry_).count();
        if (dt > 0.001 && dt <= 0.5) {
            core::WheelSpeeds wheel_mps{};
            wheel_mps.fl_mps = static_cast<double>(telemetry.delta[0]) * kMetersPerCount / dt;
            wheel_mps.fr_mps = static_cast<double>(telemetry.delta[1]) * kMetersPerCount / dt;
            wheel_mps.rl_mps = static_cast<double>(telemetry.delta[2]) * kMetersPerCount / dt;
            wheel_mps.rr_mps = static_cast<double>(telemetry.delta[3]) * kMetersPerCount / dt;
            core::integratePose(odometry_.pose, wheel_mps, dt);
        }
    }

    last_telemetry_ = now;
    odometry_.seq = telemetry.seq;
    odometry_.status = telemetry.status;
    odometry_.fault_mask = telemetry.fault_mask;
    odometry_.fresh = true;
}

std::optional<SerialBase::WheelState> SerialBase::getWheelState() const
{
    if (!last_telemetry_) {
        return std::nullopt;
    }
    if (std::chrono::steady_clock::now() - *last_telemetry_ > kTelemetryTimeout) {
        return std::nullopt;
    }
    WheelState state;
    state.counts = wheel_counts_;
    state.rpm = wheel_rpm_;
    return state;
}

std::optional<SerialBase::ImuState> SerialBase::getImu() const
{
    if (!last_telemetry_) {
        return std::nullopt;
    }
    if (std::chrono::steady_clock::now() - *last_telemetry_ > kTelemetryTimeout) {
        return std::nullopt;
    }
    return imu_;
}

std::optional<hal::OdometryState> SerialBase::getOdometry() const
{
    if (!last_telemetry_) {
        return std::nullopt;
    }
    if (std::chrono::steady_clock::now() - *last_telemetry_ > kTelemetryTimeout) {
        return std::nullopt;
    }
    return odometry_;
}

}  // namespace f401::drivers
