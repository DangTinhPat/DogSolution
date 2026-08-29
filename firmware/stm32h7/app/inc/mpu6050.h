#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>

typedef struct MPU6050_Reading
{
    float linear_acceleration[3]; /* m/s^2, sensor x/y/z */
    float angular_velocity[3];    /* rad/s, sensor x/y/z */
} MPU6050_Reading;

/* Verifies WHO_AM_I, wakes the device and selects +/-2g, +/-250 deg/s,
 * 100 Hz sampling with the MPU6050's DLPF enabled. */
bool MPU6050_Init(void);
bool MPU6050_Read(MPU6050_Reading *out);

#endif /* MPU6050_H */
