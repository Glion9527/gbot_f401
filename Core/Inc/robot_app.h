#ifndef ROBOT_APP_H
#define ROBOT_APP_H

/*
 * 小车应用层：决定“什么时候启动、向哪里走、什么时候停止”。
 * 本层把车身动作交给 mecanum，再由 motor4 为每台电机做测速和 PI 调速。
 *
 * ROBOT_APP_MODE 在编译时选择功能，修改后需要重新编译、下载：
 *   0：按住 USER 做单轮开环测试，松手停止；依次 A/FL、B/RL、C/RR、D/FR。
 *   1：原固定路线；直线 5s → 向前走左弧线至估计左转 90° → 继续直线。
 *   2/3/4/5/6：分别为后退、左平移、右平移、原地逆时针转、原地顺时针转。
 *   7：当前默认遥控模式；Mac串口中转到ROS 2，蓝色USER仅用于本地停止。
 * 模式 1..6 都要完整短按并松开 USER，再等 1s 才启动；延迟时再按取消，
 * 运行时再按停止。上电时按住按钮不会直接启动。
 *
 * MODE1 路线的 12s 从运动命令被接受开始计算，包含最初 1s 的加速，
 * 不包含启动前等待的 1s；转弯占用其中一部分时间，剩余时间继续直线。
 * 90° 仅由编码器和底盘几何尺寸估算，没有陀螺仪测量车头真实朝向；
 * 麦轮需按 X 型安装，实际地面打滑或尺寸误差仍需通过落地测试核对。
 */
#ifndef ROBOT_APP_MODE
#define ROBOT_APP_MODE       7
#endif
#define ROBOT_APP_TEST_PWM   150    /* 仅模式 0：PWM 千分比，150 = 15% */
#if ROBOT_APP_MODE == 1
#define ROBOT_APP_TEST_RPM   85.0f  /* 路线直行时四轮的目标转速，单位 RPM（转/分） */
#else
#define ROBOT_APP_TEST_RPM   15.0f  /* 模式 2..6 的低速动作测试转速 */
#endif

/* 模式 1 的起步参数；ms 表示毫秒，1000ms = 1s。 */
#define ROBOT_APP_START_RPM  15.0f  /* 起步目标，随后线性增加到 85RPM */
#define ROBOT_APP_RAMP_MS    1000U  /* 起步加速用时，不是阻塞等待 */
#define ROBOT_APP_FIRST_STRAIGHT_MS 5000U /* 包含起步加速的第一段直行时间 */

/* 实测底盘尺寸，统一用 mm；L/W 都量轮子中心，不是车板边缘。 */
#define ROBOT_APP_WHEEL_DIAMETER_MM 80.0f  /* D：轮子直径 */
#define ROBOT_APP_WHEELBASE_MM      185.0f /* L：前后轮中心距 */
#define ROBOT_APP_TRACK_MM          220.0f /* W：左右轮中心距 */

/* 左弧线控制：正角度表示从上方看逆时针，即左转。 */
#define ROBOT_APP_TURN_DEG          90.0f /* 相对弯道开始时车头方向的目标角度 */
#define ROBOT_APP_ARC_TURN_RPM      27.5f /* 最大转向 RPM 分量，不是车身角速度 */
#define ROBOT_APP_ARC_SLOWDOWN_DEG  20.0f /* 剩余角度小于此值时逐渐减小转向分量 */
#define ROBOT_APP_ARC_TOLERANCE_DEG 1.0f  /* 进入目标 ±1° 区间视为到达候选状态 */
#define ROBOT_APP_ARC_RAMP_MS       300U  /* 转向分量从 0 到最大值的最短变化时间 */
#define ROBOT_APP_ARC_SETTLE_MS     150U  /* 连续保持在误差区间内才进入第二段直行 */

#define ROBOT_APP_DEBOUNCE_MS 30U        /* 按下/松开均需稳定 30ms，过滤机械触点抖动 */
#define ROBOT_APP_START_DELAY_MS 1000U   /* 完整点击后的启动等待时间 */
#if ROBOT_APP_MODE == 0
#define ROBOT_APP_RUN_MS     5000U       /* 单轮一次最多运行 5s */
#elif ROBOT_APP_MODE == 1
#define ROBOT_APP_RUN_MS     12000U      /* 整条路线总时限；即使转弯未完成也会停机 */
#else
#define ROBOT_APP_RUN_MS     10000U      /* 其余动作测试一次运行 10s */
#endif
#define ROBOT_APP_PRINT_MS   200U        /* 串口每 200ms 打印一次，不是 PI 控制周期 */

/* main 中初始化外设和 motor4 后调用一次。模式7启动接收并等待显式使能；
 * 模式0..6进入等待按钮松开状态。 */
void RobotApp_Init(void);
/* main 的 while(1) 中反复调用；10ms 的电机控制仍由 TIM9 中断执行。 */
void RobotApp_Poll(void);

#endif
