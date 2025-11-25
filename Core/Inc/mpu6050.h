/*
 * mpu6050.h
 *
 *  Created on: Nov 25, 2025
 *      Author: MONSTER
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C 7-bit adresi 0x68, HAL ile kullanmak için 1 bit sola kaydırıyoruz
#define MPU6050_I2C_ADDR        (0x68 << 1)

// Register adresleri (en sık kullanılanlar)
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

// Ölçek çarpanları (±2g, ±250 dps için)
#define MPU6050_ACC_SENS_2G          16384.0f   // LSB/g
#define MPU6050_GYRO_SENS_250DPS     131.0f     // LSB/(°/s)

// Ham okuma struct'ı
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6050_Raw_t;

// İşlenmiş veri + açı bilgisi
typedef struct
{
    float ax_g, ay_g, az_g;         // g cinsinden ivme
    float gx_dps, gy_dps, gz_dps;   // °/s cinsinden gyro
    float pitch_deg;                // complementary filter sonucu
    float roll_deg;
} MPU6050_Proc_t;

/**
 * @brief  MPU6050'i başlatır (sleep'ten çıkarır, ±2g & ±250 dps ayarlar).
 * @param  hi2c  MPU6050'in bağlı olduğu I2C handle (örn: &hi2c1)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Tüm ham sensör verilerini okur (accel, gyro, sıcaklık).
 * @param  hi2c   I2C handle
 * @param  raw    Ham veri struct'ı
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *raw);

/**
 * @brief  Ham veriyi okuyup scale eder ve complementary filter ile
 *         pitch/roll hesaplar.
 * @param  hi2c   I2C handle
 * @param  dt     Saniye cinsinden zaman farkı (ör: 0.02f)
 * @param  out    İşlenmiş veri ve açı sonuçları
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU6050_ReadProcessed(I2C_HandleTypeDef *hi2c,
                                        float dt,
                                        MPU6050_Proc_t *out);

#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_H_ */
