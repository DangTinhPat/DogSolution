#include "main_bot_hardware/real_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/logging.hpp>

namespace main_bot_hardware
{

namespace
{
rclcpp::Logger Logger() { return rclcpp::get_logger("main_bot_hardware"); }
constexpr double kTauFfAbsLimitNm = 10.0;
constexpr uint16_t kAllJointsMask = (1U << 12U) - 1U;
constexpr auto kJointFbTimeout = std::chrono::milliseconds(100);

// Kep + kiem tra NaN/inf TRUOC khi ep kieu sang int16_t/uint16_t - ep kieu 1 double
// vuot pham vi bieu dien (hoac NaN) sang kieu nguyen la undefined behavior trong
// C++ (KHONG phai wrap an toan), khong phai loi ly thuyet: 1 loi YAML (vd nhap do
// thay vi radian cho stand_pos/sit_pos) hoac 1 buoc tinh IK/Tff sau nay tra ve gia
// tri bat thuong se toi thang day truoc khi gui xuong dong co that. Day la lop
// phong ve DAU TIEN (truoc ca lop kep KP/KD tren firmware, xem motor_calib.h's
// MOTOR_KP_ABS_LIMIT/MOTOR_KD_ABS_LIMIT va Actuator_SetTarget()'s kep vi tri).
int16_t ToInt16Safe(const double value)
{
  if (!std::isfinite(value)) { return 0; }
  return static_cast<int16_t>(std::clamp(value, -32767.0, 32767.0));
}

uint16_t ToUint16Safe(const double value)
{
  if (!std::isfinite(value)) { return 0; }
  return static_cast<uint16_t>(std::clamp(value, 0.0, 65535.0));
}
}  // namespace

hardware_interface::CallbackReturn RealSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      Logger(), "RealSystem can dung dung %u khop, xacro khai bao %zu", kJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 5U);
  for (uint32_t i = 0; i < kJointCount; i++)
  {
    interfaces.emplace_back(info_.joints[i].name, "position", &position_state_[i]);
    interfaces.emplace_back(info_.joints[i].name, "velocity", &velocity_state_[i]);
    interfaces.emplace_back(info_.joints[i].name, "effort", &effort_state_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "visual_position", &visual_position_state_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "visual_velocity", &visual_velocity_state_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RealSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount * 5U);
  for (uint32_t i = 0; i < kJointCount; i++)
  {
    interfaces.emplace_back(info_.joints[i].name, "position", &position_command_[i]);
    interfaces.emplace_back(info_.joints[i].name, "velocity", &velocity_command_[i]);
    interfaces.emplace_back(info_.joints[i].name, "kp", &kp_command_[i]);
    interfaces.emplace_back(info_.joints[i].name, "kd", &kd_command_[i]);
    interfaces.emplace_back(info_.joints[i].name, "effort", &effort_command_[i]);
  }
  return interfaces;
}

void RealSystem::JointFbCallback(const main_bot_hardware_msgs::msg::JointFb & msg)
{
  std::lock_guard<std::mutex> lock(joint_fb_mutex_);
  latest_joint_fb_ = msg;
  latest_joint_fb_time_ = std::chrono::steady_clock::now();
  has_joint_fb_ = true;
}

void RealSystem::JointDiagCallback(const main_bot_hardware_msgs::msg::JointDiag & msg)
{
  const bool changed = !has_joint_diag_ || msg.ready_mask != last_ready_mask_ ||
    msg.runtime_fault_mask != last_runtime_fault_mask_ ||
    msg.can_bus_off_mask != last_can_bus_off_mask_;
  if (!changed) { return; }

  if (msg.runtime_fault_mask != 0U || msg.can_bus_off_mask != 0U)
  {
    RCLCPP_ERROR(
      Logger(), "MCU runtime fault=0x%03x, CAN bus-off=0x%02x, ready=0x%03x",
      static_cast<unsigned int>(msg.runtime_fault_mask),
      static_cast<unsigned int>(msg.can_bus_off_mask),
      static_cast<unsigned int>(msg.ready_mask));
  }
  else if (msg.ready_mask != kAllJointsMask)
  {
    RCLCPP_WARN(
      Logger(), "MCU chi co ready mask 0x%03x/0x%03x",
      static_cast<unsigned int>(msg.ready_mask), static_cast<unsigned int>(kAllJointsMask));
  }
  else if (has_joint_diag_)
  {
    RCLCPP_INFO(Logger(), "MCU/CAN da tro lai trang thai ready");
  }

  has_joint_diag_ = true;
  last_ready_mask_ = msg.ready_mask;
  last_runtime_fault_mask_ = msg.runtime_fault_mask;
  last_can_bus_off_mask_ = msg.can_bus_off_mask;
}

hardware_interface::CallbackReturn RealSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // NaN co chu dich: truoc /joint_fb dau tien KHONG duoc gia mao q=qd=0 la
  // feedback that. StateHoldPose kiem tra isfinite() va giu Kp/Tff=0 cho toi
  // khi MCU da tra encoder hop le.
  position_state_.fill(std::numeric_limits<double>::quiet_NaN());
  velocity_state_.fill(std::numeric_limits<double>::quiet_NaN());
  effort_state_.fill(0.0);
  // HOME is logical zero. These values are only mapped into /joint_states for
  // TF/RViz and hold their last finite sample if feedback later becomes stale.
  visual_position_state_.fill(0.0);
  visual_velocity_state_.fill(0.0);
  position_command_.fill(0.0);
  velocity_command_.fill(0.0);
  kp_command_.fill(0.0);
  kd_command_.fill(0.0);
  effort_command_.fill(0.0);
  {
    std::lock_guard<std::mutex> lock(joint_fb_mutex_);
    has_joint_fb_ = false;
    latest_joint_fb_ = main_bot_hardware_msgs::msg::JointFb{};
    latest_joint_fb_time_ = std::chrono::steady_clock::time_point{};
  }
  has_joint_diag_ = false;
  last_ready_mask_ = 0U;
  last_runtime_fault_mask_ = 0U;
  last_can_bus_off_mask_ = 0U;

  // get_node(): rclcpp::Node do chinh hardware_interface framework tao + spin cho moi
  // hardware component (xem real_system.hpp) - khong tu quan ly executor/thread rieng.
  joint_cmd_pub_ = get_node()->create_publisher<main_bot_hardware_msgs::msg::JointCmd>(
    "/joint_cmd", rclcpp::SensorDataQoS());
  joint_fb_sub_ = get_node()->create_subscription<main_bot_hardware_msgs::msg::JointFb>(
    "/joint_fb", rclcpp::SensorDataQoS(),
    [this](const main_bot_hardware_msgs::msg::JointFb & msg) { JointFbCallback(msg); });
  joint_diag_sub_ = get_node()->create_subscription<main_bot_hardware_msgs::msg::JointDiag>(
    "/joint_diag", rclcpp::SensorDataQoS(),
    [this](const main_bot_hardware_msgs::msg::JointDiag & msg) { JointDiagCallback(msg); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  main_bot_hardware_msgs::msg::JointFb fb;
  bool got_fb = false;
  {
    std::lock_guard<std::mutex> lock(joint_fb_mutex_);
    if (has_joint_fb_)
    {
      fb = latest_joint_fb_;
      got_fb = (std::chrono::steady_clock::now() - latest_joint_fb_time_) < kJointFbTimeout;
    }
  }

  for (uint32_t i = 0; i < kJointCount; i++)
  {
    if (got_fb && (fb.valid_mask & (1U << i)) != 0U)
    {
      position_state_[i] = static_cast<double>(fb.measured_angle_mrad[i]) / 1000.0;
      velocity_state_[i] = static_cast<double>(fb.measured_velocity_mrad_s[i]) / 1000.0;
      effort_state_[i] = static_cast<double>(fb.measured_effort_mnm[i]) / 1000.0;
      visual_position_state_[i] = position_state_[i];
      visual_velocity_state_[i] = velocity_state_[i];
    }
    else
    {
      position_state_[i] = std::numeric_limits<double>::quiet_NaN();
      velocity_state_[i] = std::numeric_limits<double>::quiet_NaN();
      effort_state_[i] = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  main_bot_hardware_msgs::msg::JointCmd cmd;
  for (uint32_t i = 0; i < kJointCount; i++)
  {
    cmd.target_angle_mrad[i] = ToInt16Safe(position_command_[i] * 1000.0);
    cmd.target_velocity_mrad_s[i] = ToInt16Safe(velocity_command_[i] * 1000.0);
    cmd.kp_x100[i] = ToUint16Safe(kp_command_[i] * 100.0);
    cmd.kd_x100[i] = ToUint16Safe(kd_command_[i] * 100.0);
    const double tau_ff_nm = std::isfinite(effort_command_[i])
      ? std::clamp(effort_command_[i], -kTauFfAbsLimitNm, kTauFfAbsLimitNm)
      : 0.0;
    cmd.tau_ff_mnm[i] = ToInt16Safe(tau_ff_nm * 1000.0);
  }
  cmd.seq = seq_++;

  joint_cmd_pub_->publish(cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace main_bot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(main_bot_hardware::RealSystem, hardware_interface::SystemInterface)
