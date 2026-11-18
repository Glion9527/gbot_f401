/* 运动学/里程计纯算法测试：不需要串口、树莓派或 STM32。 */

#include <cmath>

#include "f401/core/mecanum.hpp"
#include "f401/core/odometry.hpp"
#include "test_support.hpp"

namespace {

using namespace f401::core;

void testStraightAndRotation()
{
    const WheelSpeeds straight = toWheelSpeeds({0.2, 0.0, 0.0});
    CHECK_NEAR(straight.fl_mps, 0.2, 1e-12);
    CHECK_NEAR(straight.fr_mps, 0.2, 1e-12);
    CHECK_NEAR(straight.rl_mps, 0.2, 1e-12);
    CHECK_NEAR(straight.rr_mps, 0.2, 1e-12);

    const WheelSpeeds spin = toWheelSpeeds({0.0, 0.0, 0.5});
    const double rotation = (kHalfWheelbaseM + kHalfTrackM) * 0.5;
    CHECK_NEAR(spin.fl_mps, -rotation, 1e-12);
    CHECK_NEAR(spin.fr_mps, rotation, 1e-12);
    CHECK_NEAR(spin.rl_mps, -rotation, 1e-12);
    CHECK_NEAR(spin.rr_mps, rotation, 1e-12);
}

void testRoundTrip()
{
    const ChassisSpeeds original{0.12, -0.05, 0.3};
    const ChassisSpeeds back = toChassisSpeeds(toWheelSpeeds(original));
    CHECK_NEAR(back.vx_mps, original.vx_mps, 1e-12);
    CHECK_NEAR(back.vy_mps, original.vy_mps, 1e-12);
    CHECK_NEAR(back.wz_radps, original.wz_radps, 1e-12);
}

void testLimits()
{
    const ChassisSpeeds clamped = clampChassisSpeeds({1.0, -1.0, 1.0});
    CHECK_NEAR(clamped.vx_mps, 0.3, 1e-12);
    CHECK_NEAR(clamped.vy_mps, -0.15, 1e-12);
    CHECK_NEAR(clamped.wz_radps, 0.6, 1e-12);

    const WheelSpeeds limited = limitWheelSpeeds({0.4, 0.4, 0.4, 0.4},
                                                 kRemoteWheelLinearLimitMps);
    CHECK_NEAR(limited.fl_mps, kRemoteWheelLinearLimitMps, 1e-12);
    CHECK_NEAR(limited.rr_mps, kRemoteWheelLinearLimitMps, 1e-12);

    const WheelSpeeds invalid = limitWheelSpeeds({std::nan(""), 0.0, 0.0, 0.0}, 0.3);
    CHECK_NEAR(invalid.fl_mps, 0.0, 0.0);
}

void testOdometry()
{
    Pose2D straight{};
    integratePose(straight, {0.1, 0.1, 0.1, 0.1}, 1.0);
    CHECK_NEAR(straight.x_m, 0.1, 1e-12);
    CHECK_NEAR(straight.y_m, 0.0, 1e-12);
    CHECK_NEAR(straight.yaw_rad, 0.0, 1e-12);

    const double rotation = (kHalfWheelbaseM + kHalfTrackM) * 0.5;
    Pose2D spin{};
    integratePose(spin, {-rotation, rotation, -rotation, rotation}, 1.0);
    CHECK_NEAR(spin.yaw_rad, 0.5, 1e-9);
    CHECK_NEAR(spin.x_m, 0.0, 1e-9);
    CHECK_NEAR(spin.y_m, 0.0, 1e-9);

    CHECK_NEAR(normalizeAngle(3.0 * kPi), kPi, 1e-12);
    CHECK_NEAR(normalizeAngle(-3.0 * kPi), kPi, 1e-12);
}

}  // namespace

int main()
{
    testStraightAndRotation();
    testRoundTrip();
    testLimits();
    testOdometry();
    return test::finish("core kinematics/odometry");
}
