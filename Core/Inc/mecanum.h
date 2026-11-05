#ifndef MECANUM_H
#define MECANUM_H

#include <stdbool.h>

/* 麦克纳姆轮运动分解层：把“小车想怎样运动”换算成四个轮子的目标转速，
 * 再交给 motor4.c 分别调速；本文件不直接操作 GPIO，也不计算电机 PWM。
 *
 * 要求轮子滚子按标准 X 形安装。从车顶往下看，车头朝上：
 *          A/FL 左前  \       /  D/FR 右前
 *          B/RL 左后  /       \  C/RR 右后
 * 斜线表示滚子的轴向，不是电机旋转方向。四轮的逻辑正转均应推动小车向前。
 * 函数参数始终按逻辑顺序 FL、FR、RL、RR，不能当成驱动板 A、B、C、D 顺序。
 *
 * 三个输入是叠加到轮子上的转速分量，单位都是 RPM（每分钟转数）：
 * forward_rpm > 0：前进；left_rpm > 0：向左平移；ccw_rpm > 0：俯视逆时针左转。
 * 它们不是车身的 m/s 或 rad/s。需要物理速度单位时使用下面的接口。
 * 任一路混合结果超过最大允许转速时，四轮一起按相同比例缩小，保留速度比例。
 * 返回 false 表示命令未接受，原命令可能仍在运行，调用方需要处理返回值。
 * 本层只下发目标；应用仍须持续确认运行条件并喂 motor4 的软件看门狗。
 */
bool Mecanum_SetMixRPM(float forward_rpm, float left_rpm, float ccw_rpm);

/* 使用车身物理速度的接口，轴向正方向与上面相同：
 * forward_m_s / left_m_s：前进/左移速度，单位 m/s；ccw_rad_s：角速度，单位 rad/s。
 * wheel_radius_m：轮子半径；half_length_m：前后轮中心距的一半；
 * half_width_m：左右轮中心距的一半。尺寸均以米为单位，必须为正数。
 * 当前底盘对应半径 0.040m、半长 0.0925m、半宽 0.110m。
 * 注意传入的是半径和半距，不能直接传直径 80 或完整轮距 185、220。
 * 计算结果仍会等比例限速，过大的车身速度指令不会原样实现。
 */
bool Mecanum_SetBodyVelocity(float forward_m_s, float left_m_s, float ccw_rad_s,
                            float wheel_radius_m,
                            float half_length_m, float half_width_m);

/* 撤销四轮驱动，实际停稳还需要车轮消耗惯性；不清累计编码器位置。 */
void Mecanum_Stop(void);

#endif
