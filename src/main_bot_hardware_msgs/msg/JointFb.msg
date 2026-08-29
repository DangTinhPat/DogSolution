# firmware/stm32h7 (publisher "/joint_fb") -> RealSystem::read(), qua micro-ROS/UART.
# valid_mask bit i=1 chi khi telemetry cua joint i con moi; RealSystem dat state thanh
# NaN neu bit bi xoa de controller khong tiep tuc dung feedback cu.
int16[12] measured_angle_mrad
int16[12] measured_velocity_mrad_s
int16[12] measured_effort_mnm
uint16 valid_mask
