# Compact MCU -> EC IMU transport. Fixed-size integer fields keep the
# micro-ROS/XRCE payload small and allocation-free on the STM32.
uint8 STATUS_OK=1
uint8 STATUS_INIT_FAILED=2
uint8 STATUS_READ_FAILED=4

# SI values scaled by 1000. MPU6050 is configured for +/-2g and +/-250 deg/s,
# so every valid sample fits comfortably in int16.
int16[3] linear_acceleration_milli_ms2
int16[3] angular_velocity_mrad_s

# Free-running STM32 Tick_GetMs() timestamp. It is used for delta-time only;
# the EC stamps /imu/data with ROS time because this is not a synchronized epoch.
uint32 stamp_ms
uint8 status
