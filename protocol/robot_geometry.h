#ifndef F401_PROTOCOL_ROBOT_GEOMETRY_H
#define F401_PROTOCOL_ROBOT_GEOMETRY_H

/*
 * 底盘几何、编码器和限速常量（宿主侧使用）。
 *
 * 这些值必须与固件保持一致：
 *   - Core/Inc/robot_app.h：D=80mm、L=185mm、W=220mm；
 *   - Core/Inc/motor4.h：CPR=3960、底层目标上限 110RPM；
 *   - Core/Src/remote_control.c：遥控轮速上限 75RPM、vx<=300mm/s、vy<=150mm/s、wz<=600mrad/s。
 *
 * 固件目前用 mm 整数宏，宿主用米；这里只保存“源头数值”的毫米形式，
 * 避免两边各自维护一套浮点常量。改接线/换轮子时需要同步修改本文件和固件。
 */

/* 轮子直径 D：80mm。 */
#define F401_WHEEL_DIAMETER_MM 80.0

/* 前后轮中心距 L=185mm，左右轮中心距 W=220mm；这里存半值，正好用于麦轮公式。 */
#define F401_HALF_WHEELBASE_MM 92.5
#define F401_HALF_TRACK_MM 110.0

/* 减速箱输出轴转一圈的编码器计数（四倍频）。 */
#define F401_ENCODER_CPR 3960.0

/* 底层（motor4）允许的最大目标轮速；超过会被固件拒绝。 */
#define F401_MAX_WHEEL_RPM 110.0

/* 当前遥控阶段的轮速上限（等比例限幅）。 */
#define F401_REMOTE_MAX_WHEEL_RPM 75.0

/* 遥控协议的车身轴输入上限。 */
#define F401_MAX_LINEAR_MM_S 300.0
#define F401_MAX_STRAFE_MM_S 150.0
#define F401_MAX_ANGULAR_MRAD_S 600.0

#endif /* F401_PROTOCOL_ROBOT_GEOMETRY_H */
