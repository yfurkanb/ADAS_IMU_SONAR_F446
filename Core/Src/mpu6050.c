/*
 * mpu6050.c
 *
 *  Created on: Nov 25, 2025
 *      Author: MONSTER
 */

#include "mpu6050.h"
#include <math.h>
#include <string.h>

// Complementary filter için internal state
static float s_pitch_deg = 0.0f;
static float s_roll_deg  = 0.0f;
static uint8_t s_angles_initialized = 0;

// Küçük helper fonksiyonlar
static HAL_StatusTypeDef mpu6050_write_reg(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg,
                                           uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c,
                             MPU6050_I2C_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1,
                             100);
}

static HAL_StatusTypeDef mpu6050_read_regs(I2C_HandleTypeDef *hi2c,
                                           uint8_t start_reg,
                                           uint8_t *pData,
                                           uint16_t size)
{
    return HAL_I2C_Mem_Read(hi2c,
                            MPU6050_I2C_ADDR,
                            start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            pData,
                            size,
                            100);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check = 0;
    HAL_StatusTypeDef ret;

    // WHO_AM_I kontrolü (0x68 dönmeli)
    ret = mpu6050_read_regs(hi2c, MPU6050_REG_WHO_AM_I, &check, 1);
    if (ret != HAL_OK)
        return ret;

    if (check != 0x68)
        return HAL_ERROR;

    // Sleep'ten çık (PWR_MGMT_1 = 0x00 → internal 8MHz, sleep disabled)
    ret = mpu6050_write_reg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != HAL_OK) return ret;

    // Gyro config: ±250 dps (FS_SEL = 0) → 0x00
    ret = mpu6050_write_reg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != HAL_OK) return ret;

    // Accel config: ±2g (AFS_SEL = 0) → 0x00
    ret = mpu6050_write_reg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != HAL_OK) return ret;

    // Low-pass filter & sample rate ayarları basit bırakıldı
    // CONFIG = 0x03 → ~44 Hz DLPF, GYRO
    ret = mpu6050_write_reg(hi2c, MPU6050_REG_CONFIG, 0x03);
    if (ret != HAL_OK) return ret;

    // Sample rate divider: SMPLRT_DIV = 0 → 1 kHz/(1+0) = 1 kHz (DLPF ile sınırlanır)
    ret = mpu6050_write_reg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (ret != HAL_OK) return ret;

    // Complementary filter state reset
    s_pitch_deg = 0.0f;
    s_roll_deg  = 0.0f;
    s_angles_initialized = 0;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *raw)
{
    uint8_t buf[14];
    HAL_StatusTypeDef ret;

    if (raw == NULL)
        return HAL_ERROR;

    ret = mpu6050_read_regs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != HAL_OK)
        return ret;

    // Big-endian -> int16
    raw->ax   = (int16_t)((buf[0] << 8) | buf[1]);
    raw->ay   = (int16_t)((buf[2] << 8) | buf[3]);
    raw->az   = (int16_t)((buf[4] << 8) | buf[5]);
    raw->temp = (int16_t)((buf[6] << 8) | buf[7]);
    raw->gx   = (int16_t)((buf[8] << 8) | buf[9]);
    raw->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadProcessed(I2C_HandleTypeDef *hi2c,
                                        float dt,
                                        MPU6050_Proc_t *out)
{
    MPU6050_Raw_t raw;
    HAL_StatusTypeDef ret;

    if (out == NULL)
        return HAL_ERROR;

    ret = MPU6050_ReadRaw(hi2c, &raw);
    if (ret != HAL_OK)
        return ret;

    // 1) Scale et
    float ax = (float)raw.ax / MPU6050_ACC_SENS_2G;
    float ay = (float)raw.ay / MPU6050_ACC_SENS_2G;
    float az = (float)raw.az / MPU6050_ACC_SENS_2G;

    float gx = (float)raw.gx / MPU6050_GYRO_SENS_250DPS; // °/s
    float gy = (float)raw.gy / MPU6050_GYRO_SENS_250DPS;
    float gz = (float)raw.gz / MPU6050_GYRO_SENS_250DPS;

    out->ax_g = ax;
    out->ay_g = ay;
    out->az_g = az;
    out->gx_dps = gx;
    out->gy_dps = gy;
    out->gz_dps = gz;

    // 2) Accelerometer'dan pitch/roll hesabı
    // roll  = atan2(ay, az)
    // pitch = atan2(-ax, sqrt(ay^2 + az^2))
    float roll_acc  = atan2f(ay, az) * 180.0f / (float)M_PI;
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

    const float alpha = 0.98f;  // gyro %98, accel %2

    if (!s_angles_initialized)
    {
        s_pitch_deg = pitch_acc;
        s_roll_deg  = roll_acc;
        s_angles_initialized = 1;
    }
    else
    {
        // 3) Gyro entegrasyonu
        float pitch_gyro = s_pitch_deg + gx * dt;
        float roll_gyro  = s_roll_deg  + gy * dt;

        // 4) Complementary filter
        s_pitch_deg = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
        s_roll_deg  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    }

    out->pitch_deg = s_pitch_deg;
    out->roll_deg  = s_roll_deg;

    return HAL_OK;
}
