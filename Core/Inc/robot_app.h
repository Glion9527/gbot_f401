#ifndef ROBOT_APP_H
#define ROBOT_APP_H

/*
 * MODE 0: open-loop one wheel per press, cycling A/FL, B/RL, C/RR, D/FR.
 * Modes 1..6: short press and release USER, wait 1 second, run 10 seconds.
 * Press USER again during the delay to cancel, or during motion to stop.
 * MODE 1 forward (default), 2 backward, 3 left, 4 right, 5 CCW, 6 CW.
 * MODE 1 starts at 15 RPM and ramps to 85 RPM over the first running second.
 * The 10-second run includes that ramp; modes 2..6 remain at 15 RPM.
 * MODE 0 retains the hold-to-run per-wheel wiring test (maximum 5 seconds).
 * Initial tests: securely support the chassis with all wheels off the table.
 */
#define ROBOT_APP_MODE       1
#define ROBOT_APP_TEST_PWM   150
#if ROBOT_APP_MODE == 1
#define ROBOT_APP_TEST_RPM   85.0f
#else
#define ROBOT_APP_TEST_RPM   15.0f
#endif
#define ROBOT_APP_START_RPM  15.0f
#define ROBOT_APP_RAMP_MS    1000U
#define ROBOT_APP_DEBOUNCE_MS 30U
#define ROBOT_APP_START_DELAY_MS 1000U
#if ROBOT_APP_MODE == 0
#define ROBOT_APP_RUN_MS     5000U
#else
#define ROBOT_APP_RUN_MS     10000U
#endif
#define ROBOT_APP_PRINT_MS   200U

void RobotApp_Init(void);
void RobotApp_Poll(void);

#endif
