/**
 * icm20948.c -- ICM-20948(9 轴) + BMP280 驱动
 *
 * 硬件：微雪 10 DOF IMU (D)，实测为克隆件：
 *   - ICM 六轴 WHO_AM_I=0xEA，寄存器按 ICM-20948 用；
 *   - 磁力计是克隆 AK09916（WIA1=0x48、WIA2=0x09），数据在 0x11、低字节在前；
 *   - BMP280 地址 0x77（SDO 拉高），驱动先试 0x76 再试 0x77。
 *
 * 踩过的坑：
 *   1) 配置寄存器在 bank2（0x7F 切 bank），不是 bank0；
 *   2) 磁力计必须开 ICM 旁路 INT_PIN_CFG.BYPASS_EN；
 *   3) 磁力计数据小端，ICM 加速度/陀螺大端；
 *   4) AK09916 数据读完必须读 ST2(0x18)，否则不刷新（这里一次读 9 字节）。
 *
 * I2C：使用 CubeMX 生成的 hi2c1（Core/Src/i2c.c），本文件不做 I2C 初始化。
 */

#include "i2c.h"
#include "icm20948.h"
#include <math.h>
#include <string.h>

#define ICM_ADDR (0x68U << 1)
#define MAG_ADDR (0x0CU << 1)

/* bank 0 */
#define R_WHO_AM_I     0x00
#define R_USER_CTRL    0x03
#define R_PWR_MGMT_1   0x06
#define R_PWR_MGMT_2   0x07
#define R_INT_PIN_CFG  0x0F
#define R_ACCEL_XOUT_H 0x2D
#define R_TEMP_OUT_H   0x39
#define R_REG_BANK_SEL 0x7F
/* bank 2 */
#define R_GYRO_SMPLRT_DIV    0x00
#define R_GYRO_CONFIG_1      0x01
#define R_ACCEL_SMPLRT_DIV_1 0x10
#define R_ACCEL_SMPLRT_DIV_2 0x11
#define R_ACCEL_CONFIG       0x14
/* AK09916 */
#define R_MAG_WIA    0x00
#define R_MAG_CNTL2  0x31
#define R_MAG_CNTL3  0x32
#define R_MAG_DATA   0x10   /* ST1 + 6 字节数据 + TMPS + ST2 */
/* BMP280 */
#define R_BMP_ID   0xD0
#define R_BMP_CAL  0x88
#define R_BMP_CTRL 0xF4
#define R_BMP_CFG  0xF5
#define R_BMP_DATA 0xF7

#define ACCEL_LSB_PER_G   8192.0f   /* ±4g  */
#define GYRO_LSB_PER_DPS  65.5f     /* ±500dps */
#define MAG_LSB_PER_UT    0.15f

#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.2957795f

#define COMP_ALPHA   0.98f  /* 互补滤波：陀螺权重 */
#define YAW_MAG_GAIN 0.02f  /* 每个采样把 yaw 往磁力计拉 2% */

static uint8_t  s_bank = 0xFF;
static uint8_t  s_icm_ok, s_mag_ok, s_bmp_ok, s_inited;
static uint16_t s_bmp_addr = (0x77U << 1);
static float    s_gyro_bias[3];
static float    s_mag_off[3];
static float    s_roll, s_pitch, s_yaw, s_yaw_offset;
static uint32_t s_last_tick;
static uint32_t s_bmp_tick;
static float    s_press_hpa, s_alt_m, s_temp_c;

/* 磁力计硬铁标定用的极值 */
static uint8_t s_mag_cal;
static float   s_mag_min[3], s_mag_max[3];

/* ------------------------------------------------------------------ I2C 底层 */
static HAL_StatusTypeDef rd(uint16_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 20);
}

static HAL_StatusTypeDef wr(uint16_t dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 20);
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static HAL_StatusTypeDef icm_set_bank(uint8_t bank)
{
    HAL_StatusTypeDef st;

    if (bank == s_bank) {
        return HAL_OK;
    }
    st = wr(ICM_ADDR, R_REG_BANK_SEL, (uint8_t)(bank << 4));
    if (st == HAL_OK) {
        s_bank = bank;
    }
    return st;
}

/* --------------------------------------------------------------- ICM 初始化 */
static uint8_t icm_config(void)
{
    uint8_t id = 0, v = 0;

    if (rd(ICM_ADDR, R_WHO_AM_I, &id, 1) != HAL_OK || id != 0xEA) {
        return 0;
    }

    (void)wr(ICM_ADDR, R_PWR_MGMT_1, 0x80);   /* DEVICE_RESET */
    HAL_Delay(100);
    s_bank = 0xFF;
    (void)icm_set_bank(0);

    (void)wr(ICM_ADDR, R_PWR_MGMT_1, 0x01);   /* 唤醒，自动时钟 */
    (void)wr(ICM_ADDR, R_PWR_MGMT_2, 0x00);   /* 六轴全开 */
    HAL_Delay(10);

    (void)icm_set_bank(2);
    (void)wr(ICM_ADDR, R_GYRO_CONFIG_1, 0x1B);        /* ±500dps, DLPF 51Hz */
    (void)wr(ICM_ADDR, R_ACCEL_CONFIG, 0x1B);         /* ±4g,    DLPF 50Hz */
    (void)wr(ICM_ADDR, R_GYRO_SMPLRT_DIV, 4);         /* 1125/(1+4)=225Hz */
    (void)wr(ICM_ADDR, R_ACCEL_SMPLRT_DIV_1, 0x00);
    (void)wr(ICM_ADDR, R_ACCEL_SMPLRT_DIV_2, 4);
    (void)icm_set_bank(0);

    (void)wr(ICM_ADDR, R_USER_CTRL, 0x00);            /* 关 ICM 内部 I2C 主机 */
    (void)rd(ICM_ADDR, R_INT_PIN_CFG, &v, 1);
    (void)wr(ICM_ADDR, R_INT_PIN_CFG, (uint8_t)(v | 0x02)); /* 开磁力计旁路 */
    HAL_Delay(50);

    return 1;
}

/* ------------------------------------------------------------- 磁力计初始化 */
static uint8_t mag_config(void)
{
    uint8_t id1 = 0, id2 = 0;

    if (rd(MAG_ADDR, R_MAG_WIA, &id1, 1) != HAL_OK) {
        return 0;
    }
    (void)rd(MAG_ADDR, 0x01, &id2, 1);

    /* 0x09/0x0C = AK09916/AK09918；0x48+WIA2=0x09 = 本模块的克隆片 */
    if (!(id1 == 0x09 || id1 == 0x0C || (id1 == 0x48 && id2 == 0x09))) {
        return 0;
    }

    if (id1 != 0x48) {
        (void)wr(MAG_ADDR, R_MAG_CNTL3, 0x01);   /* 软复位 */
        HAL_Delay(100);
    }
    (void)wr(MAG_ADDR, R_MAG_CNTL2, 0x08);       /* 100Hz 连续测量 */
    HAL_Delay(10);
    return 1;
}

/* --------------------------------------------------------------- BMP280 部分 */
static uint16_t dig_T1, dig_P1;
static int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

static uint8_t bmp_config(void)
{
    static const uint16_t cand[2] = {(0x76U << 1), (0x77U << 1)};
    uint8_t c[24], id = 0, i;

    for (i = 0; i < 2; i++) {
        if (rd(cand[i], R_BMP_ID, &id, 1) == HAL_OK && id == 0x58) {
            s_bmp_addr = cand[i];
            break;
        }
    }
    if (i == 2) {
        return 0;
    }
    if (rd(s_bmp_addr, R_BMP_CAL, c, 24) != HAL_OK) {
        return 0;
    }
    dig_T1 = (uint16_t)(((uint16_t)c[1] << 8) | c[0]);
    dig_T2 = (int16_t)(((uint16_t)c[3] << 8) | c[2]);
    dig_T3 = (int16_t)(((uint16_t)c[5] << 8) | c[4]);
    dig_P1 = (uint16_t)(((uint16_t)c[7] << 8) | c[6]);
    dig_P2 = (int16_t)(((uint16_t)c[9] << 8) | c[8]);
    dig_P3 = (int16_t)(((uint16_t)c[11] << 8) | c[10]);
    dig_P4 = (int16_t)(((uint16_t)c[13] << 8) | c[12]);
    dig_P5 = (int16_t)(((uint16_t)c[15] << 8) | c[14]);
    dig_P6 = (int16_t)(((uint16_t)c[17] << 8) | c[16]);
    dig_P7 = (int16_t)(((uint16_t)c[19] << 8) | c[18]);
    dig_P8 = (int16_t)(((uint16_t)c[21] << 8) | c[20]);
    dig_P9 = (int16_t)(((uint16_t)c[23] << 8) | c[22]);

    (void)wr(s_bmp_addr, R_BMP_CTRL, 0x27);   /* 温度 x1、气压 x1、normal */
    (void)wr(s_bmp_addr, R_BMP_CFG, 0x00);
    HAL_Delay(20);
    return 1;
}

static void bmp_read(void)
{
    uint8_t d[6];
    int32_t adc_P, adc_T;
    double var1, var2, p;

    if (rd(s_bmp_addr, R_BMP_DATA, d, 6) != HAL_OK) {
        return;
    }
    adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);

    var1 = ((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0;
    var1 = var1 * ((double)dig_T2);
    var2 = ((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0;
    var2 = var2 * var2 * ((double)dig_T3);
    t_fine = (int32_t)(var1 + var2);
    s_temp_c = (float)((var1 + var2) / 5120.0);

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = var2 / 4.0 + ((double)dig_P4) * 65536.0;
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
    if (var1 == 0.0) {
        return;
    }
    p = 1048576.0 - (double)adc_P;
    p = (p - var2 / 4096.0) * 6250.0 / var1;
    var1 = ((double)dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;

    s_press_hpa = (float)(p / 100.0);
    s_alt_m = 44330.0f * (1.0f - powf(s_press_hpa / 1013.25f, 0.1903f));
}

/* ------------------------------------------------------------------ 读传感器 */
static uint8_t read_icm(icm_data_t *d)
{
    uint8_t b[14];

    if (rd(ICM_ADDR, R_ACCEL_XOUT_H, b, 12) != HAL_OK) {
        return 0;
    }
    d->acc_g[0] = be16(b + 0) / ACCEL_LSB_PER_G;
    d->acc_g[1] = be16(b + 2) / ACCEL_LSB_PER_G;
    d->acc_g[2] = be16(b + 4) / ACCEL_LSB_PER_G;
    d->gyro_dps[0] = be16(b + 6) / GYRO_LSB_PER_DPS;
    d->gyro_dps[1] = be16(b + 8) / GYRO_LSB_PER_DPS;
    d->gyro_dps[2] = be16(b + 10) / GYRO_LSB_PER_DPS;

    if (rd(ICM_ADDR, R_TEMP_OUT_H, b, 2) == HAL_OK) {
        d->temp_c = be16(b) / 333.87f + 21.0f;
    }
    return 1;
}

static void read_mag(icm_data_t *d)
{
    uint8_t m[9];
    float mx, my, mz, norm;

    d->mag_ok = 0;
    if (!s_mag_ok || rd(MAG_ADDR, R_MAG_DATA, m, 9) != HAL_OK) {
        return;
    }
    /* 数据低字节在前，单位 0.15uT/LSB */
    mx = le16(m + 1) * MAG_LSB_PER_UT;
    my = le16(m + 3) * MAG_LSB_PER_UT;
    mz = le16(m + 5) * MAG_LSB_PER_UT;
    if (mx == 0.0f && my == 0.0f && mz == 0.0f) {
        return;
    }

    if (s_mag_cal) {   /* 硬铁标定：记录每个轴能到的最小/最大值 */
        if (mx < s_mag_min[0]) s_mag_min[0] = mx;
        if (my < s_mag_min[1]) s_mag_min[1] = my;
        if (mz < s_mag_min[2]) s_mag_min[2] = mz;
        if (mx > s_mag_max[0]) s_mag_max[0] = mx;
        if (my > s_mag_max[1]) s_mag_max[1] = my;
        if (mz > s_mag_max[2]) s_mag_max[2] = mz;
    }

    mx -= s_mag_off[0];
    my -= s_mag_off[1];
    mz -= s_mag_off[2];
    d->mag_uT[0] = mx;
    d->mag_uT[1] = my;
    d->mag_uT[2] = mz;

    norm = sqrtf(mx * mx + my * my + mz * mz);
    d->mag_ok = (norm > 15.0f && norm < 120.0f) ? 1 : 0;
}

static float wrap180(float a)
{
    while (a > 180.0f) {
        a -= 360.0f;
    }
    while (a < -180.0f) {
        a += 360.0f;
    }
    return a;
}

static void update_attitude(icm_data_t *d, float dt)
{
    float roll_acc, pitch_acc;
    float sr, cr, sp, cp, mxh, myh, yaw_mag, err;

    roll_acc = atan2f(d->acc_g[1], d->acc_g[2]) * RAD2DEG;
    pitch_acc = atan2f(-d->acc_g[0], sqrtf(d->acc_g[1] * d->acc_g[1] + d->acc_g[2] * d->acc_g[2])) * RAD2DEG;

    /* 互补滤波：陀螺短时准，加速度计长期准 */
    s_roll = COMP_ALPHA * (s_roll + d->gyro_dps[0] * dt) + (1.0f - COMP_ALPHA) * roll_acc;
    s_pitch = COMP_ALPHA * (s_pitch + d->gyro_dps[1] * dt) + (1.0f - COMP_ALPHA) * pitch_acc;

    /* 航向：陀螺积分 + 磁力计慢修正（只修正倾角补偿后的水平分量） */
    s_yaw += d->gyro_dps[2] * dt;
    if (d->mag_ok) {
        sr = sinf(s_roll * DEG2RAD);
        cr = cosf(s_roll * DEG2RAD);
        sp = sinf(s_pitch * DEG2RAD);
        cp = cosf(s_pitch * DEG2RAD);
        mxh = d->mag_uT[0] * cp + d->mag_uT[1] * sr * sp + d->mag_uT[2] * cr * sp;
        myh = d->mag_uT[1] * cr - d->mag_uT[2] * sr;
        yaw_mag = -atan2f(myh, mxh) * RAD2DEG;  /* 与陀螺同向（逆时针为正）；实测反了就去掉负号 */
        err = wrap180(yaw_mag - s_yaw);
        s_yaw += YAW_MAG_GAIN * err;
    }
    s_yaw = wrap180(s_yaw);
    s_roll = wrap180(s_roll);

    d->roll_deg = s_roll;
    d->pitch_deg = s_pitch;
    d->yaw_deg = wrap180(s_yaw - s_yaw_offset);
}

/* ------------------------------------------------------------------ 对外接口 */
void ICM20948_ZeroYaw(void)
{
    s_yaw_offset = s_yaw;
}

void ICM20948_CalibrateGyro(uint32_t sample_ms)
{
    int32_t sum[3] = {0, 0, 0};
    uint32_t n = 0, t0 = HAL_GetTick();
    uint8_t b[12];

    while ((uint32_t)(HAL_GetTick() - t0) < sample_ms) {
        if (rd(ICM_ADDR, R_ACCEL_XOUT_H, b, 12) == HAL_OK) {
            sum[0] += be16(b + 6);
            sum[1] += be16(b + 8);
            sum[2] += be16(b + 10);
            n++;
        }
        HAL_Delay(4);
    }
    if (n > 0) {
        s_gyro_bias[0] = ((float)sum[0] / (float)n) / GYRO_LSB_PER_DPS;
        s_gyro_bias[1] = ((float)sum[1] / (float)n) / GYRO_LSB_PER_DPS;
        s_gyro_bias[2] = ((float)sum[2] / (float)n) / GYRO_LSB_PER_DPS;
    }
}

void ICM20948_SetMagOffset(const float off_uT[3])
{
    s_mag_off[0] = off_uT[0];
    s_mag_off[1] = off_uT[1];
    s_mag_off[2] = off_uT[2];
}

void ICM20948_MagCalStart(void)
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        s_mag_min[i] = 1e6f;
        s_mag_max[i] = -1e6f;
    }
    s_mag_cal = 1;
}

uint8_t ICM20948_MagCalFinish(float off_uT[3])
{
    uint8_t i;

    if (!s_mag_cal) {
        return 0;
    }
    s_mag_cal = 0;
    for (i = 0; i < 3; i++) {
        if (s_mag_max[i] - s_mag_min[i] < 50.0f) {   /* 该轴没转到，标定无效 */
            return 0;
        }
        off_uT[i] = (s_mag_max[i] + s_mag_min[i]) * 0.5f;
        s_mag_off[i] = off_uT[i];
    }
    return 1;
}

uint8_t ICM20948_Init(void)
{
    memset(&s_gyro_bias, 0, sizeof(s_gyro_bias));
    memset(&s_mag_off, 0, sizeof(s_mag_off));
    s_roll = s_pitch = s_yaw = 0.0f;
    s_mag_cal = 0;
    s_bmp_tick = 0;

    s_icm_ok = icm_config();
    if (!s_icm_ok) {
        return 0;
    }
    s_mag_ok = mag_config();
    s_bmp_ok = bmp_config();

    /* 上电静止 0.8s 估陀螺零偏；这期间别动车 */
    ICM20948_CalibrateGyro(800);
    bmp_read();
    s_last_tick = HAL_GetTick();
    s_inited = 1;
    return 1;
}

uint8_t ICM20948_Read(icm_data_t *out)
{
    uint32_t now, dt_ms;
    uint8_t i;

    if (!s_inited || out == NULL) {
        return 0;
    }

    if (!read_icm(out)) {
        return 0;
    }
    for (i = 0; i < 3; i++) {
        out->gyro_dps[i] -= s_gyro_bias[i];
    }
    read_mag(out);

    now = HAL_GetTick();
    dt_ms = now - s_last_tick;
    s_last_tick = now;
    if (dt_ms == 0 || dt_ms > 200) {
        dt_ms = 20;   /* 首次/卡顿后给个合理步长 */
    }

    if ((now - s_bmp_tick) >= 100U && s_bmp_ok) {   /* 气压 10Hz 足够 */
        s_bmp_tick = now;
        bmp_read();
    }
    out->press_hpa = s_press_hpa;
    out->alt_m = s_alt_m;
    out->temp_c = (s_bmp_ok && s_temp_c > -40.0f) ? s_temp_c : out->temp_c;

    update_attitude(out, (float)dt_ms / 1000.0f);
    out->bmp_ok = s_bmp_ok;
    out->timestamp_ms = now;
    return 1;
}
