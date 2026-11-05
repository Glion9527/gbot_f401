#include "mecanum.h"
#include "motor4.h"
#include <math.h>

/* 对四个轮子统一限速后，一次性下发，避免出现只更新一部分轮子的情况。 */
static bool SetScaled(double fl, double fr, double rl, double rr)
{
    double wheel[4] = {fl, fr, rl, rr};
    double peak = 0.0;

    for (unsigned i = 0; i < 4; ++i) {
        /* NaN/Inf 是无效数值，不能交给后面的 PI 控制或浮点转整数操作。 */
        if (!isfinite(wheel[i])) {
            return false;
        }
        double magnitude = fabs(wheel[i]);
        if (magnitude > peak) {
            peak = magnitude;
        }
    }

    /* 例如最大轮速算出 220RPM、上限 110RPM，就把四路都乘 0.5。
     * 只截断最快的一路会破坏四轮比例，使实际行驶方向/转弯曲率发生变化。 */
    double scale = peak > (double)MOTOR4_MAX_TARGET_RPM
                 ? (double)MOTOR4_MAX_TARGET_RPM / peak : 1.0;

    return Motor4_SetAllSpeedsRPM((float)(fl * scale), (float)(fr * scale),
                                 (float)(rl * scale), (float)(rr * scale));
}

bool Mecanum_SetMixRPM(float forward_rpm, float left_rpm, float ccw_rpm)
{
    /* 上层路线/动作测试使用本接口；三个参数可以混合，但每个参数必须是有限数。
     * “向前15、向左0、左转0”表示轮速分量，不表示小车向前15米/秒。 */
    if (!isfinite(forward_rpm) || !isfinite(left_rpm) ||
        !isfinite(ccw_rpm)) {
        return false;
    }

    /* float 输入先转 double，避免多个很大的有限 float 相加时溢出。
     * X 型轮组的混合公式（以下顺序为 FL、FR、RL、RR）：
     *   左前 = 前进 - 左移 - 左转；右前 = 前进 + 左移 + 左转；
     *   左后 = 前进 + 左移 - 左转；右后 = 前进 - 左移 + 左转。
     * 只给 f 时四轮同速前进；只给 t 时左侧反转、右侧正转，原地左转；
     * 同时给 f 和 t 可以边前进边转弯，是否反转取决于两者大小。 */
    double f = forward_rpm;
    double l = left_rpm;
    double t = ccw_rpm;
    return SetScaled(f - l - t, f + l + t,
                     f + l - t, f - l + t);
}

bool Mecanum_SetBodyVelocity(float forward_m_s, float left_m_s, float ccw_rad_s,
                            float wheel_radius_m,
                            float half_length_m, float half_width_m)
{
    /* 遥控层使用此接口：例如串口vx=80mm/s会先变成forward_m_s=0.08。
     * 正的几何尺寸是换算的前提，尤其轮半径位于除数中，不能为0。 */
    if (!isfinite(forward_m_s) || !isfinite(left_m_s) ||
        !isfinite(ccw_rad_s) || !isfinite(wheel_radius_m) ||
        !isfinite(half_length_m) || !isfinite(half_width_m) ||
        wheel_radius_m <= 0.0f || half_length_m <= 0.0f ||
        half_width_m <= 0.0f) {
        return false;
    }

    /* 轮缘线速度 v = 角速度 * 半径 r，因此轮子角速度 = v/r。
     * 再用 1转 = 2π弧度、1分钟 = 60秒，把 rad/s 换算成 RPM。
     * 转向分量还要乘 (半长 + 半宽)，体现底盘尺寸对旋转轮速的影响。 */
    const double rpm_per_rad_s = 60.0 / (2.0 * 3.14159265358979323846);
    double factor = rpm_per_rad_s / (double)wheel_radius_m;
    double f = (double)forward_m_s * factor;
    double l = (double)left_m_s * factor;
    double t = ((double)half_length_m + (double)half_width_m)
             * (double)ccw_rad_s * factor;

    return SetScaled(f - l - t, f + l + t,
                     f + l - t, f - l + t);
}

void Mecanum_Stop(void)
{
    /* 运动层只提供统一接口，具体清目标、撤销STBY和保留位置等由电机层完成。 */
    Motor4_StopAll();
}
