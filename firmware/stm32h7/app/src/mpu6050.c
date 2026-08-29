#include "mpu6050.h"

#include "app_i2c.h"
#include "tick.h"

#include <stddef.h>

#define MPU6050_ADDR7             0x68U
#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_WHO_AM_I_VALUE    0x68U
#define ACCEL_LSB_PER_G           16384.0f
#define GYRO_LSB_PER_DPS          131.0f
#define STANDARD_GRAVITY          9.80665f
#define DEG_TO_RAD                0.01745329252f

static int16_t bytes_to_i16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static void wait_ms(uint32_t duration_ms)
{
    const uint32_t start_ms = Tick_GetMs();
    while ((Tick_GetMs() - start_ms) < duration_ms)
    {
    }
}

bool MPU6050_Init(void)
{
    uint8_t who_am_i = 0U;
    if (!AppI2c1_ReadRegs(MPU6050_ADDR7, MPU6050_REG_WHO_AM_I, &who_am_i, 1U) ||
        (who_am_i != MPU6050_WHO_AM_I_VALUE))
    {
        return false;
    }

    /* PLL with X gyro reference, DLPF cfg=3 (~42 Hz gyro/~44 Hz accel),
     * internal 1 kHz rate / (1 + 9) = 100 Hz. Ranges are explicit rather
     * than relying on power-on defaults. */
    if (!AppI2c1_WriteReg(MPU6050_ADDR7, MPU6050_REG_PWR_MGMT_1, 0x01U))
    {
        return false;
    }
    wait_ms(10U);
    return AppI2c1_WriteReg(MPU6050_ADDR7, MPU6050_REG_CONFIG, 0x03U) &&
           AppI2c1_WriteReg(MPU6050_ADDR7, MPU6050_REG_SMPLRT_DIV, 0x09U) &&
           AppI2c1_WriteReg(MPU6050_ADDR7, MPU6050_REG_GYRO_CONFIG, 0x00U) &&
           AppI2c1_WriteReg(MPU6050_ADDR7, MPU6050_REG_ACCEL_CONFIG, 0x00U);
}

bool MPU6050_Read(MPU6050_Reading *out)
{
    uint8_t raw[14];
    if ((out == NULL) ||
        !AppI2c1_ReadRegs(MPU6050_ADDR7, MPU6050_REG_ACCEL_XOUT_H,
                          raw, (uint8_t)sizeof(raw)))
    {
        return false;
    }

    const int16_t ax = bytes_to_i16(raw[0], raw[1]);
    const int16_t ay = bytes_to_i16(raw[2], raw[3]);
    const int16_t az = bytes_to_i16(raw[4], raw[5]);
    const int16_t gx = bytes_to_i16(raw[8], raw[9]);
    const int16_t gy = bytes_to_i16(raw[10], raw[11]);
    const int16_t gz = bytes_to_i16(raw[12], raw[13]);

    out->linear_acceleration[0] = ((float)ax / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    out->linear_acceleration[1] = ((float)ay / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    out->linear_acceleration[2] = ((float)az / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    out->angular_velocity[0] = ((float)gx / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    out->angular_velocity[1] = ((float)gy / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    out->angular_velocity[2] = ((float)gz / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    return true;
}
