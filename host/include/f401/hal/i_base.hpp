#pragma once

/*
 * 底盘硬件抽象接口（HAL）。
 *
 * 技能/任务层只依赖 IBase，不依赖串口、帧格式或具体电机。
 * 这样同一个上层程序可以注入：
 *   - SerialBase：真实串口 + STM32；
 *   - SimBase：没有硬件时的运动学仿真。
 *
 * 线程模型约定：
 *   - 当前骨架是单线程：主循环依次调用 poll()、setVelocity()、getOdometry()；
 *   - 后续阶段若改成独立线程，实现必须自行加锁，并在类注释中写明；
 *   - 接口本身不隐藏锁，避免调用方误以为可以跨线程随意调用。
 */

#include <cstdint>
#include <optional>

#include "f401/core/mecanum.hpp"
#include "f401/core/odometry.hpp"

namespace f401::hal {

struct OdometryState {
    core::Pose2D pose{};      // 航迹推算位姿（米/弧度）
    std::uint16_t seq = 0;    // 最近一帧遥测的序号
    std::uint16_t status = 0; // STM32 状态位（LINK_V2_STATUS_*）
    std::uint16_t fault_mask = 0;
    bool fresh = false;       // 数据是否在有效期内
};

class IBase {
public:
    virtual ~IBase() = default;

    // 下发车身速度指令。返回 false 表示这次写入失败（链路断开、参数异常）。
    virtual bool setVelocity(const core::ChassisSpeeds& cmd) = 0;

    // 读取最近里程计。没有数据或超过有效期时返回 nullopt。
    virtual std::optional<OdometryState> getOdometry() const = 0;

    // 处理一次待处理的 I/O。单线程骨架由主循环周期调用；
    // 未来线程化实现可以让它变成空操作。
    virtual void poll() {}
};

}  // namespace f401::hal
