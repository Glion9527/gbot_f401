#pragma once

/*
 * 麦克纳姆轮运动学（纯算法层）。
 *
 * 这个头文件故意不依赖串口、ROS、线程或任何硬件：
 *   - 可以在 Mac 上单独编译、写单元测试；
 *   - 将来可以整体搬进 ROS 2 节点或固件（只要换掉 include 路径）。
 *
 * 轮序固定为 FL、FR、RL、RR；速度符号约定：
 *   vx > 0 前进，vy > 0 向左，wz > 0 俯视逆时针。
 * 轮速正方向统一为“推动小车前进”。
 *
 * 公式与 Core/Src/mecanum.c 一致：
 *   FL = f - l - t
 *   FR = f + l + t
 *   RL = f + l - t
 *   RR = f - l + t
 * 其中 t = (半轴距 + 半轮距) × wz。
 */

#include <algorithm>
#include <cmath>
#include <numbers>

#include "robot_geometry.h"

namespace f401::core {

struct ChassisSpeeds {
    double vx_mps = 0.0;
    double vy_mps = 0.0;
    double wz_radps = 0.0;
};

struct WheelSpeeds {
    double fl_mps = 0.0;
    double fr_mps = 0.0;
    double rl_mps = 0.0;
    double rr_mps = 0.0;
};

/* 几何常量从共享契约换算到米。 */
inline constexpr double kPi = std::numbers::pi;
inline constexpr double kWheelRadiusM = F401_WHEEL_DIAMETER_MM / 2000.0;
inline constexpr double kHalfWheelbaseM = F401_HALF_WHEELBASE_MM / 1000.0;
inline constexpr double kHalfTrackM = F401_HALF_TRACK_MM / 1000.0;

/* 遥控阶段允许的单轮最大线速度（75RPM 对应的轮缘线速度）。 */
inline constexpr double kRemoteWheelLinearLimitMps =
    (F401_REMOTE_MAX_WHEEL_RPM / 60.0) * kPi * (F401_WHEEL_DIAMETER_MM / 1000.0);

/* 逆运动学：车身速度 -> 四轮轮缘线速度（m/s）。 */
inline WheelSpeeds toWheelSpeeds(const ChassisSpeeds& chassis)
{
    const double rotation = (kHalfWheelbaseM + kHalfTrackM) * chassis.wz_radps;
    return WheelSpeeds{
        chassis.vx_mps - chassis.vy_mps - rotation,
        chassis.vx_mps + chassis.vy_mps + rotation,
        chassis.vx_mps + chassis.vy_mps - rotation,
        chassis.vx_mps - chassis.vy_mps + rotation,
    };
}

/* 正运动学：四轮轮缘线速度 -> 车身速度；用于里程计。 */
inline ChassisSpeeds toChassisSpeeds(const WheelSpeeds& wheel)
{
    return ChassisSpeeds{
        0.25 * (wheel.fl_mps + wheel.fr_mps + wheel.rl_mps + wheel.rr_mps),
        0.25 * (-wheel.fl_mps + wheel.fr_mps + wheel.rl_mps - wheel.rr_mps),
        (-wheel.fl_mps + wheel.fr_mps - wheel.rl_mps + wheel.rr_mps) /
            (4.0 * (kHalfWheelbaseM + kHalfTrackM)),
    };
}

/* 在进入运动学前做一次轴限幅。比例缩放（而不是逐轴截断）留到 limitWheelSpeeds，
 * 这样急停/异常值不会产生一个方向被截断、另一个方向保留的“歪”指令。 */
inline ChassisSpeeds clampChassisSpeeds(const ChassisSpeeds& cmd)
{
    return ChassisSpeeds{
        std::clamp(cmd.vx_mps, -F401_MAX_LINEAR_MM_S / 1000.0, F401_MAX_LINEAR_MM_S / 1000.0),
        std::clamp(cmd.vy_mps, -F401_MAX_STRAFE_MM_S / 1000.0, F401_MAX_STRAFE_MM_S / 1000.0),
        std::clamp(cmd.wz_radps, -F401_MAX_ANGULAR_MRAD_S / 1000.0, F401_MAX_ANGULAR_MRAD_S / 1000.0),
    };
}

/* 四轮等比例限幅：保持方向/曲率，不单独截断某一路。
 * NaN/Inf 直接返回全零，防止异常值进入 PI/电机。 */
inline WheelSpeeds limitWheelSpeeds(const WheelSpeeds& wheel, double limit_mps)
{
    const double values[4] = {wheel.fl_mps, wheel.fr_mps, wheel.rl_mps, wheel.rr_mps};
    double peak = 0.0;
    for (double value : values) {
        if (!std::isfinite(value)) {
            return WheelSpeeds{};
        }
        peak = std::max(peak, std::fabs(value));
    }
    if (peak <= limit_mps || peak <= 0.0) {
        return wheel;
    }
    const double scale = limit_mps / peak;
    return WheelSpeeds{wheel.fl_mps * scale, wheel.fr_mps * scale, wheel.rl_mps * scale,
                       wheel.rr_mps * scale};
}

}  // namespace f401::core
