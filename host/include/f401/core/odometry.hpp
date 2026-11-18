#pragma once

/*
 * 编码器里程计（纯算法层）。
 *
 * 输入是“一个控制周期内的四轮轮缘线速度”（m/s），输出位姿积分。
 * 这里只做正运动学 + 航向积分，不依赖 IMU；后续阶段可以在上层做 EKF 融合。
 *
 * 注意：这是“航迹推算”，麦轮打滑、轮径/CPR 标定误差都会累积。
 * 真正的家庭导航需要激光/视觉重定位，本模块只提供短时相对运动。
 */

#include <cmath>

#include "f401/core/mecanum.hpp"

namespace f401::core {

struct Pose2D {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

/* 把角度归一化到 (-pi, pi]，避免长时间旋转后数值发散。 */
inline double normalizeAngle(double angle)
{
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

/* 用中点航向积分：先算本周期车体位移，再按半个航向变化旋转到世界系。
 * dt <= 0 或非有限值时什么都不做，避免异常时间戳污染位姿。 */
inline void integratePose(Pose2D& pose, const WheelSpeeds& wheel_mps, double dt_s)
{
    if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
        return;
    }
    const ChassisSpeeds chassis = toChassisSpeeds(wheel_mps);
    if (!std::isfinite(chassis.vx_mps) || !std::isfinite(chassis.vy_mps) ||
        !std::isfinite(chassis.wz_radps)) {
        return;
    }
    const double delta_yaw = chassis.wz_radps * dt_s;
    const double mid_yaw = pose.yaw_rad + 0.5 * delta_yaw;
    pose.x_m += (chassis.vx_mps * std::cos(mid_yaw) - chassis.vy_mps * std::sin(mid_yaw)) * dt_s;
    pose.y_m += (chassis.vx_mps * std::sin(mid_yaw) + chassis.vy_mps * std::cos(mid_yaw)) * dt_s;
    pose.yaw_rad = normalizeAngle(pose.yaw_rad + delta_yaw);
}

}  // namespace f401::core
