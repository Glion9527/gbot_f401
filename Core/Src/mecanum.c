#include "mecanum.h"
#include "motor4.h"
#include <math.h>

static bool SetScaled(double fl, double fr, double rl, double rr)
{
    double wheel[4] = {fl, fr, rl, rr};
    double peak = 0.0;

    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(wheel[i])) {
            return false;
        }
        double magnitude = fabs(wheel[i]);
        if (magnitude > peak) {
            peak = magnitude;
        }
    }

    double scale = peak > (double)MOTOR4_MAX_TARGET_RPM
                 ? (double)MOTOR4_MAX_TARGET_RPM / peak : 1.0;

    return Motor4_SetAllSpeedsRPM((float)(fl * scale), (float)(fr * scale),
                                 (float)(rl * scale), (float)(rr * scale));
}

bool Mecanum_SetMixRPM(float forward_rpm, float left_rpm, float ccw_rpm)
{
    if (!isfinite(forward_rpm) || !isfinite(left_rpm) ||
        !isfinite(ccw_rpm)) {
        return false;
    }

    /* Double intermediates prevent overflow when summing finite float inputs. */
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
    if (!isfinite(forward_m_s) || !isfinite(left_m_s) ||
        !isfinite(ccw_rad_s) || !isfinite(wheel_radius_m) ||
        !isfinite(half_length_m) || !isfinite(half_width_m) ||
        wheel_radius_m <= 0.0f || half_length_m <= 0.0f ||
        half_width_m <= 0.0f) {
        return false;
    }

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
    Motor4_StopAll();
}
