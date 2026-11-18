/**
 * icm20948.h -- ICM-20948(9 轴) + BMP280 驱动（本工程的克隆模块也支持）
 *
 * 接线：VCC->3V3  GND->GND  SDA->PB9  SCL->PB8（CubeMX 的 I2C1）
 *
 * 用法：
 *   icm_data_t imu;
 *   ICM20948_Init();                   // 复位、配置量程、标定陀螺零偏(约 0.8s，需静止)
 *   while (1) { ICM20948_Read(&imu); } // 20ms 一次；roll/pitch/yaw 单位是度
 *
 * 量程：加速度 ±4g(8192 LSB/g)，陀螺 ±500dps(65.5 LSB/dps)，DLPF ~50Hz。
 *
 * I2C 由 CubeMX 生成的 MX_I2C1_Init() / hi2c1 提供，本驱动不重复初始化。
 */
#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 0 = 整个 IMU 不初始化、不读、不打印；不用 IMU 时改成 0 重新编译 */
#define IMU_DRIVER_ENABLE 0

/* 串口日志开关：
 *   0 = 不把姿态文本打印到 USART2（推荐）。控制串口只跑遥控协议帧，
 *       避免 IMU 文本占用主循环/带宽，或与 T 帧拼接造成网关 CRC 失败。
 *   1 = 每 100ms 打一行 "IMU r=... p=... y=..."，仅用于台架调试。
 * IMU 数据仍然照常读取，调试时用 CubeIDE 的 Live Watch 查看。 */
#define IMU_LOG_ENABLE 0

typedef struct {
    float acc_g[3];      /* 加速度，单位 g */
    float gyro_dps[3];   /* 角速度，已减零偏，单位 deg/s */
    float mag_uT[3];     /* 磁场，已减硬铁偏置，单位 uT */
    float temp_c;        /* 温度，摄氏度 */
    float press_hpa;     /* 气压，hPa */
    float alt_m;         /* 相对海平面高度（按 1013.25 hPa 基准），m */
    float roll_deg;      /* 横滚，-180..180 */
    float pitch_deg;     /* 俯仰，-90..90 */
    float yaw_deg;       /* 航向，-180..180（磁力计修正后的陀螺积分） */
    uint8_t mag_ok;      /* 1 = 本次磁力计数据可用 */
    uint32_t timestamp_ms;
} icm_data_t;

/* 初始化。返回 1 成功（ICM 应答），0 失败。耗时约 0.9s，期间板子必须静止。 */
uint8_t ICM20948_Init(void);

/* 读一次并更新姿态。返回 1 成功。 */
uint8_t ICM20948_Read(icm_data_t *out);

/* 把当前航向记为 0°（之后 yaw 相对这个方向）。上车后先摆正再调用。 */
void ICM20948_ZeroYaw(void);

/* 重新标定陀螺零偏，期间必须静止。典型 800ms。 */
void ICM20948_CalibrateGyro(uint32_t sample_ms);

/* 手动设置磁力计硬铁偏置（uT）。一般用下面的两点标定，不用手填。 */
void ICM20948_SetMagOffset(const float off_uT[3]);

/* 磁力计硬铁标定：调用 Start 后把模块各方向转几圈，再调 Finish 取偏置。 */
void ICM20948_MagCalStart(void);
uint8_t ICM20948_MagCalFinish(float off_uT[3]);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */
