#pragma once

/*
 * IBase 的仿真实现：没有硬件时验证“上层 -> 运动学 -> 里程计”整条链路。
 * 它不打开串口，只按指令速度做欧拉积分，适合在 Mac 上跑通控制/行为层。
 */

#include <chrono>
#include <optional>

#include "f401/hal/i_base.hpp"
#include "link_v2.h"

namespace f401::app {

class SimBase final : public hal::IBase {
public:
    bool setVelocity(const core::ChassisSpeeds& cmd) override
    {
        command_ = cmd;
        return true;
    }

    std::optional<hal::OdometryState> getOdometry() const override { return odometry_; }

    void poll() override
    {
        const auto now = std::chrono::steady_clock::now();
        if (!last_poll_) {
            last_poll_ = now;
            return;
        }
        const double dt = std::chrono::duration<double>(now - *last_poll_).count();
        last_poll_ = now;
        const core::WheelSpeeds wheel = core::toWheelSpeeds(command_);
        core::integratePose(odometry_.pose, wheel, dt);
        ++odometry_.seq;
        odometry_.status = LINK_V2_STATUS_ARMED;
        odometry_.fresh = true;
    }

private:
    core::ChassisSpeeds command_{};
    hal::OdometryState odometry_{};
    std::optional<std::chrono::steady_clock::time_point> last_poll_;
};

}  // namespace f401::app
