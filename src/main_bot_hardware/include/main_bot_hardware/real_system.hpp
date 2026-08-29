#ifndef MAIN_BOT_HARDWARE__REAL_SYSTEM_HPP_
#define MAIN_BOT_HARDWARE__REAL_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <mutex>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <main_bot_hardware_msgs/msg/joint_cmd.hpp>
#include <main_bot_hardware_msgs/msg/joint_diag.hpp>
#include <main_bot_hardware_msgs/msg/joint_fb.hpp>

// hardware_interface::SystemInterface cho robot that - relay position/velocity/kp/kd/effort(Tff) toi
// firmware/stm32h7 qua micro-ROS/UART (topic "/joint_cmd"/"/joint_fb",
// main_bot_hardware_msgs), firmware forward xuong 12 board driver khop qua CAN-FD
// (actuator_if.c). Khac gz_ros2_control/
// GazeboSimSystem o cho export "position"/"velocity"/"kp"/"kd"/"effort" thang toi
// board driver khop (tu lam PD cuc bo + cong tau_ff).
//
// Dung get_node() (hardware_interface::HardwareComponentInterface, framework tu tao +
// tu spin cho moi hardware plugin) de publish/subscribe thang - KHONG tu quan ly
// executor/thread rieng nhu ban CAN cu (CanFdSocket can polling non-blocking thu cong),
// micro_ros_agent (chay ben ngoai, xem real_ros2_control.launch.py) la noi thuc su noi
// UART toi DDS, RealSystem chi la 1 node ROS2 binh thuong o phia nay.
namespace main_bot_hardware
{

class RealSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void JointFbCallback(const main_bot_hardware_msgs::msg::JointFb & msg);
  void JointDiagCallback(const main_bot_hardware_msgs::msg::JointDiag & msg);

  rclcpp::Publisher<main_bot_hardware_msgs::msg::JointCmd>::SharedPtr joint_cmd_pub_;
  rclcpp::Subscription<main_bot_hardware_msgs::msg::JointFb>::SharedPtr joint_fb_sub_;
  rclcpp::Subscription<main_bot_hardware_msgs::msg::JointDiag>::SharedPtr joint_diag_sub_;
  uint8_t seq_ = 0U;

  // Buffer /joint_fb moi nhat, ghi boi callback subscription (thread cua node executor),
  // doc boi read() (thread cua controller_manager's update loop) - can mutex vi 2 thread
  // khac nhau, khac CanFdSocket cu (rut het frame non-blocking ngay trong read(), khong
  // co callback bat dong bo).
  std::mutex joint_fb_mutex_;
  main_bot_hardware_msgs::msg::JointFb latest_joint_fb_;
  std::chrono::steady_clock::time_point latest_joint_fb_time_{};
  bool has_joint_fb_ = false;
  bool has_joint_diag_ = false;
  uint16_t last_ready_mask_ = 0U;
  uint16_t last_runtime_fault_mask_ = 0U;
  uint8_t last_can_bus_off_mask_ = 0U;

  // Thu tu khop voi info_.joints (tu xacro/controllers_real.yaml, PHAI dung 12 khop
  // theo dung thu tu JointIndex_t trong motor_topology.h - khong tu sap xep lai).
  static constexpr uint32_t kJointCount = 12U;
  std::array<double, kJointCount> position_state_{};
  std::array<double, kJointCount> velocity_state_{};
  // Measured torque reported by BabyAlpha2 telemetry.
  std::array<double, kJointCount> effort_state_{};
  // Last finite sample for display only. The controller continues to claim the
  // raw position/velocity interfaces above, which become NaN on stale/invalid
  // feedback and therefore retain their fail-safe semantics.
  std::array<double, kJointCount> visual_position_state_{};
  std::array<double, kJointCount> visual_velocity_state_{};
  std::array<double, kJointCount> position_command_{};
  std::array<double, kJointCount> velocity_command_{};
  std::array<double, kJointCount> kp_command_{};
  std::array<double, kJointCount> kd_command_{};
  std::array<double, kJointCount> effort_command_{};
};

}  // namespace main_bot_hardware

#endif  // MAIN_BOT_HARDWARE__REAL_SYSTEM_HPP_
