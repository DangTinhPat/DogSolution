#include "imu_kalman_filter/imu_kalman.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "main_bot_hardware_msgs/msg/imu_raw.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace imu_kalman_filter
{
namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;
constexpr double kRadiansToDegrees = 57.29577951308232;

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string vector_string(const Vector3 & value)
{
  std::ostringstream stream;
  stream << value.x << "," << value.y << "," << value.z;
  return stream.str();
}
}  // namespace

class ImuKalmanNode : public rclcpp::Node
{
public:
  ImuKalmanNode()
  : Node("imu_kalman_filter")
  {
    input_source_ = declare_parameter<std::string>("input_source", "compact");
    compact_topic_ = declare_parameter<std::string>("compact_topic", "/imu/raw");
    sensor_topic_ = declare_parameter<std::string>("sensor_topic", "/imu/sim_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    stale_timeout_seconds_ = declare_parameter<double>("stale_timeout_seconds", 0.25);
    yaw_variance_ = declare_parameter<double>("yaw_variance", 1000.0);

    FilterConfig config;
    const auto axis_map = declare_parameter<std::vector<int64_t>>(
      "axis_map", std::vector<int64_t>{0, 1, 2});
    const auto axis_sign = declare_parameter<std::vector<double>>(
      "axis_sign", std::vector<double>{1.0, 1.0, 1.0});
    const auto accel_scale = declare_parameter<std::vector<double>>(
      "accel_scale", std::vector<double>{1.0, 1.0, 1.0});
    if (axis_map.size() != 3U || axis_sign.size() != 3U || accel_scale.size() != 3U) {
      throw std::invalid_argument(
              "axis_map, axis_sign and accel_scale must each contain exactly 3 values");
    }
    for (std::size_t i = 0; i < 3U; ++i) {
      if (axis_map[i] < 0 || axis_map[i] > 2) {
        throw std::invalid_argument("axis_map entries must be in [0, 2]");
      }
      config.axis_map[i] = static_cast<int>(axis_map[i]);
      config.axis_sign[i] = axis_sign[i];
      config.accel_scale[i] = accel_scale[i];
    }
    accel_scale_ = {accel_scale[0], accel_scale[1], accel_scale[2]};
    const auto sensor_to_body_rpy = declare_parameter<std::vector<double>>(
      "sensor_to_body_rpy_rad", std::vector<double>{0.0, 0.0, 0.0});
    if (sensor_to_body_rpy.size() != 3U) {
      throw std::invalid_argument("sensor_to_body_rpy_rad must contain exactly 3 values");
    }
    for (std::size_t i = 0; i < 3U; ++i) {
      config.sensor_to_body_rpy_rad[i] = sensor_to_body_rpy[i];
    }
    sensor_to_body_rpy_rad_ = {
      sensor_to_body_rpy[0], sensor_to_body_rpy[1], sensor_to_body_rpy[2]};
    const int calibration_samples = declare_parameter<int>("calibration_samples", 200);
    if (calibration_samples <= 0) {
      throw std::invalid_argument("calibration_samples must be positive");
    }
    config.calibration_samples = static_cast<std::size_t>(calibration_samples);
    calibration_target_ = config.calibration_samples;
    config.gravity = declare_parameter<double>("gravity", 9.80665);
    config.stationary_gyro_threshold = declare_parameter<double>(
      "stationary_gyro_threshold", 0.15);
    config.stationary_accel_threshold = declare_parameter<double>(
      "stationary_accel_threshold", 0.8);
    config.minimum_dt = declare_parameter<double>("minimum_dt", 0.001);
    config.maximum_dt = declare_parameter<double>("maximum_dt", 0.1);
    config.angle_process_variance = declare_parameter<double>(
      "angle_process_variance", 0.001);
    config.bias_process_variance = declare_parameter<double>(
      "bias_process_variance", 0.003);
    config.accel_angle_measurement_variance = declare_parameter<double>(
      "accel_angle_measurement_variance", 0.03);
    config.adaptive_accel_threshold = declare_parameter<double>(
      "adaptive_accel_threshold", 0.5);
    config.adaptive_accel_gain = declare_parameter<double>(
      "adaptive_accel_gain", 4.0);
    config.adaptive_gravity_residual_threshold = declare_parameter<double>(
      "adaptive_gravity_residual_threshold", 0.35);
    config.adaptive_gravity_residual_gain = declare_parameter<double>(
      "adaptive_gravity_residual_gain", 8.0);
    config.maximum_measurement_scale = declare_parameter<double>(
      "maximum_measurement_scale", 100.0);
    config.accel_process_variance = declare_parameter<double>(
      "accel_process_variance", 0.05);
    config.accel_measurement_variance = declare_parameter<double>(
      "accel_measurement_variance", 0.25);
    config.gyro_process_variance = declare_parameter<double>(
      "gyro_process_variance", 0.001);
    config.gyro_measurement_variance = declare_parameter<double>(
      "gyro_measurement_variance", 0.01);
    accel_covariance_ = config.accel_measurement_variance;
    gyro_covariance_ = config.gyro_measurement_variance;

    if (!std::isfinite(stale_timeout_seconds_) || stale_timeout_seconds_ <= 0.0 ||
      !std::isfinite(yaw_variance_) || yaw_variance_ <= 0.0)
    {
      throw std::invalid_argument("stale_timeout_seconds and yaw_variance must be positive");
    }
    filter_ = std::make_unique<ImuKalman>(config);

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    if (input_source_ == "compact") {
      compact_subscription_ = create_subscription<main_bot_hardware_msgs::msg::ImuRaw>(
        compact_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuKalmanNode::compact_callback, this, std::placeholders::_1));
    } else if (input_source_ == "sensor_msgs") {
      sensor_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        sensor_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuKalmanNode::sensor_callback, this, std::placeholders::_1));
    } else {
      throw std::invalid_argument("input_source must be 'compact' or 'sensor_msgs'");
    }

    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&ImuKalmanNode::publish_diagnostics, this));
    RCLCPP_INFO(
      get_logger(),
      "Kalman IMU: source=%s, output=%s. Keep robot stationary for %zu samples.",
      input_source_.c_str(), output_topic_.c_str(), config.calibration_samples);
  }

private:
  void compact_callback(const main_bot_hardware_msgs::msg::ImuRaw::SharedPtr message)
  {
    mark_received();
    last_raw_status_ = message->status;
    if ((message->status & main_bot_hardware_msgs::msg::ImuRaw::STATUS_OK) == 0U) {
      state_ = "sensor_error";
      ++rejected_samples_;
      filter_->reset();
      have_compact_stamp_ = false;
      have_estimate_ = false;
      return;
    }

    if (!have_compact_stamp_) {
      have_compact_stamp_ = true;
      last_compact_stamp_ms_ = message->stamp_ms;
      compact_time_seconds_ = 0.0;
    } else {
      const uint32_t delta_ms = message->stamp_ms - last_compact_stamp_ms_;
      last_compact_stamp_ms_ = message->stamp_ms;
      compact_time_seconds_ += static_cast<double>(delta_ms) / 1000.0;
    }

    ImuSample sample;
    sample.linear_acceleration = {
      static_cast<double>(message->linear_acceleration_milli_ms2[0]) / 1000.0,
      static_cast<double>(message->linear_acceleration_milli_ms2[1]) / 1000.0,
      static_cast<double>(message->linear_acceleration_milli_ms2[2]) / 1000.0,
    };
    sample.angular_velocity = {
      static_cast<double>(message->angular_velocity_mrad_s[0]) / 1000.0,
      static_cast<double>(message->angular_velocity_mrad_s[1]) / 1000.0,
      static_cast<double>(message->angular_velocity_mrad_s[2]) / 1000.0,
    };
    sample.timestamp_seconds = compact_time_seconds_;
    process_sample(sample, now());
  }

  void sensor_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    mark_received();
    ImuSample sample;
    sample.linear_acceleration = {
      message->linear_acceleration.x,
      message->linear_acceleration.y,
      message->linear_acceleration.z,
    };
    sample.angular_velocity = {
      message->angular_velocity.x,
      message->angular_velocity.y,
      message->angular_velocity.z,
    };
    rclcpp::Time input_stamp(message->header.stamp);
    if (input_stamp.nanoseconds() == 0) {
      input_stamp = now();
    }
    sample.timestamp_seconds = input_stamp.seconds();
    process_sample(sample, input_stamp);
  }

  void mark_received()
  {
    received_sample_ = true;
    last_received_time_ = std::chrono::steady_clock::now();
    ++received_samples_;
  }

  void process_sample(const ImuSample & sample, const rclcpp::Time & output_stamp)
  {
    const UpdateResult result = filter_->update(sample);
    switch (result.status) {
      case UpdateStatus::kCalibrating:
        state_ = "calibrating";
        return;
      case UpdateStatus::kCalibrationMotion:
        state_ = "calibration_motion";
        return;
      case UpdateStatus::kInvalidSample:
        state_ = "invalid_sample";
        ++rejected_samples_;
        return;
      case UpdateStatus::kInvalidDeltaTime:
        state_ = "invalid_dt";
        ++rejected_samples_;
        return;
      case UpdateStatus::kReady:
        state_ = "ready";
        publish_imu(result.estimate, output_stamp);
        return;
    }
  }

  void publish_imu(const Estimate & estimate, const rclcpp::Time & stamp)
  {
    last_estimate_ = estimate;
    have_estimate_ = true;
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;

    const double half_roll = estimate.roll * 0.5;
    const double half_pitch = estimate.pitch * 0.5;
    const double half_yaw = estimate.yaw * 0.5;
    const double cr = std::cos(half_roll);
    const double sr = std::sin(half_roll);
    const double cp = std::cos(half_pitch);
    const double sp = std::sin(half_pitch);
    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);
    message.orientation.w = cr * cp * cy + sr * sp * sy;
    message.orientation.x = sr * cp * cy - cr * sp * sy;
    message.orientation.y = cr * sp * cy + sr * cp * sy;
    message.orientation.z = cr * cp * sy - sr * sp * cy;

    message.angular_velocity.x = estimate.angular_velocity.x;
    message.angular_velocity.y = estimate.angular_velocity.y;
    message.angular_velocity.z = estimate.angular_velocity.z;
    message.linear_acceleration.x = estimate.linear_acceleration.x;
    message.linear_acceleration.y = estimate.linear_acceleration.y;
    message.linear_acceleration.z = estimate.linear_acceleration.z;

    message.orientation_covariance.fill(0.0);
    message.orientation_covariance[0] = estimate.roll_variance;
    message.orientation_covariance[4] = estimate.pitch_variance;
    message.orientation_covariance[8] = yaw_variance_;
    message.angular_velocity_covariance.fill(0.0);
    message.linear_acceleration_covariance.fill(0.0);
    for (std::size_t index : {0U, 4U, 8U}) {
      message.angular_velocity_covariance[index] = gyro_covariance_;
      message.linear_acceleration_covariance[index] = accel_covariance_;
    }
    imu_publisher_->publish(message);
    ++published_samples_;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.name = "megaDog/imu_kalman";
    status.hardware_id = "mpu6050";

    const bool stale = received_sample_ &&
      (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_received_time_).count() >
      stale_timeout_seconds_);
    if (!received_sample_) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "No IMU samples received";
    } else if (stale) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "IMU input is stale";
    } else if (state_ == "sensor_error" || state_ == "invalid_sample") {
      status.level = DiagnosticStatus::ERROR;
      status.message = "IMU reported an invalid sample";
    } else if (!filter_->calibrated()) {
      status.level = DiagnosticStatus::WARN;
      status.message = (state_ == "calibration_motion") ?
        "Keep robot stationary; calibration restarted" : "Calibrating gyro bias";
    } else if (state_ == "invalid_dt") {
      status.level = DiagnosticStatus::WARN;
      status.message = "Rejected IMU sample with invalid delta-time";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "IMU Kalman filter healthy";
    }

    status.values.push_back(key_value("input_source", input_source_));
    status.values.push_back(key_value(
      "sensor_to_body_rpy_rad", vector_string(sensor_to_body_rpy_rad_)));
    status.values.push_back(key_value("accel_scale", vector_string(accel_scale_)));
    status.values.push_back(key_value("state", state_));
    status.values.push_back(key_value(
      "calibration_samples", std::to_string(filter_->calibration_count())));
    status.values.push_back(key_value(
      "calibration_target", std::to_string(calibration_target_)));
    status.values.push_back(key_value("received_samples", std::to_string(received_samples_)));
    status.values.push_back(key_value("published_samples", std::to_string(published_samples_)));
    status.values.push_back(key_value("rejected_samples", std::to_string(rejected_samples_)));
    status.values.push_back(key_value("raw_status", std::to_string(last_raw_status_)));
    status.values.push_back(key_value("gyro_bias_rad_s", vector_string(filter_->gyro_bias())));
    if (have_estimate_) {
      status.values.push_back(key_value(
        "roll_deg", std::to_string(last_estimate_.roll * kRadiansToDegrees)));
      status.values.push_back(key_value(
        "pitch_deg", std::to_string(last_estimate_.pitch * kRadiansToDegrees)));
      status.values.push_back(key_value(
        "yaw_deg_unobservable", std::to_string(last_estimate_.yaw * kRadiansToDegrees)));
    }
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  std::string input_source_;
  std::string compact_topic_;
  std::string sensor_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double stale_timeout_seconds_{0.25};
  double yaw_variance_{1000.0};
  double accel_covariance_{0.25};
  double gyro_covariance_{0.01};
  Vector3 sensor_to_body_rpy_rad_;
  Vector3 accel_scale_{1.0, 1.0, 1.0};
  std::size_t calibration_target_{0U};

  std::unique_ptr<ImuKalman> filter_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<main_bot_hardware_msgs::msg::ImuRaw>::SharedPtr compact_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  bool received_sample_{false};
  bool have_estimate_{false};
  bool have_compact_stamp_{false};
  uint32_t last_compact_stamp_ms_{0U};
  double compact_time_seconds_{0.0};
  std::chrono::steady_clock::time_point last_received_time_{};
  std::string state_{"waiting"};
  uint8_t last_raw_status_{0U};
  uint64_t received_samples_{0U};
  uint64_t published_samples_{0U};
  uint64_t rejected_samples_{0U};
  Estimate last_estimate_;
};

}  // namespace imu_kalman_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<imu_kalman_filter::ImuKalmanNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_kalman_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
