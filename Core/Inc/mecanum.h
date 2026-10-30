#ifndef MECANUM_H
#define MECANUM_H

#include <stdbool.h>

/*
 * Standard X roller arrangement; wheel order FL, FR, RL, RR.
 * Each wheel's positive direction must first be configured as vehicle forward.
 * All three inputs below are wheel-RPM contributions, not m/s or rad/s.
 * Positive axes: forward, left, counterclockwise viewed from above.
 * If a mixed wheel target exceeds the motor limit, all four are scaled equally.
 * This API sets targets only. The application must keep feeding the watchdog.
 */
bool Mecanum_SetMixRPM(float forward_rpm, float left_rpm, float ccw_rpm);

/*
 * Optional dimensional interface. Supply measured positive dimensions in meters.
 * half_length: center to front/rear wheel-center line.
 * half_width: center to left/right wheel-center line.
 * Limits are applied by equal scaling, so large requested body speeds are reduced.
 */
bool Mecanum_SetBodyVelocity(float forward_m_s, float left_m_s, float ccw_rad_s,
                            float wheel_radius_m,
                            float half_length_m, float half_width_m);

void Mecanum_Stop(void);

#endif
