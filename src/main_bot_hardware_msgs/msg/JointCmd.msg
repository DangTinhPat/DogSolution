# RealSystem::write() -> firmware/stm32h7 (subscriber "/joint_cmd", node
# stm32_joint_node) qua micro-ROS/UART. Moi field dieu khien la mang co dinh
# 12 khop theo cung thu tu JointIndex_t; khong duoc rut velocity/Kp/Kd/Tff ve
# mot gia tri dung chung vi hip/thigh/knee can gioi han/gain rieng.
#
# Don vi so nguyen giu message co dinh, khong cap phat dong tren MCU:
# - angle: mrad
# - velocity: mrad/s (controller kep <= 5 rad/s, du xa bien int16)
# - Kp/Kd: gain * 100
# - tau_ff: mN.m (N.m * 1000), int16 du dai ma hoa +/-24 N.m cua BabyAlpha2.
int16[12] target_angle_mrad
int16[12] target_velocity_mrad_s
uint16[12] kp_x100
uint16[12] kd_x100
int16[12] tau_ff_mnm
uint8 seq
