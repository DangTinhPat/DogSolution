#include "imu_kalman_filter/imu_kalman.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace imu_kalman_filter
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double component(const Vector3 & value, int index)
{
  if (index == 0) {
    return value.x;
  }
  if (index == 1) {
    return value.y;
  }
  return value.z;
}
}  // namespace

ImuKalman::ImuKalman(const FilterConfig & config)
: config_(config)
{
  std::array<bool, 3> used{{false, false, false}};
  int permutation_inversions = 0;
  for (std::size_t i = 0; i < 3U; ++i) {
    const int axis = config_.axis_map[i];
    if (axis < 0 || axis > 2 || used[static_cast<std::size_t>(axis)]) {
      throw std::invalid_argument("axis_map must be a permutation of [0, 1, 2]");
    }
    used[static_cast<std::size_t>(axis)] = true;
    if (!std::isfinite(config_.axis_sign[i]) ||
      std::abs(std::abs(config_.axis_sign[i]) - 1.0) > 1e-9)
    {
      throw std::invalid_argument("axis_sign entries must be +1 or -1");
    }
    for (std::size_t previous = 0; previous < i; ++previous) {
      if (config_.axis_map[previous] > axis) {
        ++permutation_inversions;
      }
    }
  }
  const double mapping_determinant =
    ((permutation_inversions % 2) == 0 ? 1.0 : -1.0) *
    config_.axis_sign[0] * config_.axis_sign[1] * config_.axis_sign[2];
  if (mapping_determinant < 0.0) {
    throw std::invalid_argument("axis_map/axis_sign must describe a right-handed rotation");
  }
  for (const double angle : config_.sensor_to_body_rpy_rad) {
    if (!std::isfinite(angle) || std::abs(angle) > kPi) {
      throw std::invalid_argument("sensor_to_body_rpy_rad entries must be finite and within +/-pi");
    }
  }

  const auto positive_finite = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
  const auto nonnegative_finite = [](double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  for (const double scale : config_.accel_scale) {
    if (!positive_finite(scale)) {
      throw std::invalid_argument("accel_scale entries must be finite and positive");
    }
  }
  if (config_.calibration_samples == 0U || !positive_finite(config_.gravity) ||
    !nonnegative_finite(config_.stationary_gyro_threshold) ||
    !nonnegative_finite(config_.stationary_accel_threshold) ||
    !positive_finite(config_.minimum_dt) ||
    !positive_finite(config_.maximum_dt) ||
    config_.maximum_dt <= config_.minimum_dt ||
    !positive_finite(config_.angle_process_variance) ||
    !positive_finite(config_.bias_process_variance) ||
    !positive_finite(config_.accel_angle_measurement_variance) ||
    !positive_finite(config_.adaptive_accel_threshold) ||
    !nonnegative_finite(config_.adaptive_accel_gain) ||
    !positive_finite(config_.adaptive_gravity_residual_threshold) ||
    !nonnegative_finite(config_.adaptive_gravity_residual_gain) ||
    !std::isfinite(config_.maximum_measurement_scale) ||
    config_.maximum_measurement_scale < 1.0 ||
    !positive_finite(config_.accel_process_variance) ||
    !positive_finite(config_.accel_measurement_variance) ||
    !positive_finite(config_.gyro_process_variance) ||
    !positive_finite(config_.gyro_measurement_variance))
  {
    throw std::invalid_argument("invalid non-positive Kalman configuration");
  }

  roll_filter_.configure(
    config_.angle_process_variance, config_.bias_process_variance,
    config_.accel_angle_measurement_variance);
  pitch_filter_.configure(
    config_.angle_process_variance, config_.bias_process_variance,
    config_.accel_angle_measurement_variance);
  for (auto & filter : accel_filters_) {
    filter.configure(config_.accel_process_variance, config_.accel_measurement_variance);
  }
  for (auto & filter : gyro_filters_) {
    filter.configure(config_.gyro_process_variance, config_.gyro_measurement_variance);
  }
  reset();
}

void ImuKalman::AngleBiasFilter::configure(
  double q_angle, double q_bias, double r_measure)
{
  q_angle_ = q_angle;
  q_bias_ = q_bias;
  r_measure_ = r_measure;
}

void ImuKalman::AngleBiasFilter::reset(double angle, double bias)
{
  angle_ = angle;
  bias_ = bias;
  p00_ = 0.0;
  p01_ = 0.0;
  p10_ = 0.0;
  p11_ = 0.0;
}

double ImuKalman::AngleBiasFilter::update(
  double measured_angle, double gyro_rate, double dt, double measurement_scale)
{
  const double unbiased_rate = gyro_rate - bias_;
  angle_ = ImuKalman::normalize_angle(angle_ + dt * unbiased_rate);

  p00_ += dt * (dt * p11_ - p01_ - p10_ + q_angle_);
  p01_ -= dt * p11_;
  p10_ -= dt * p11_;
  p11_ += q_bias_ * dt;

  const double innovation_variance = p00_ + r_measure_ * measurement_scale;
  const double gain_angle = p00_ / innovation_variance;
  const double gain_bias = p10_ / innovation_variance;
  const double innovation = ImuKalman::normalize_angle(measured_angle - angle_);

  angle_ = ImuKalman::normalize_angle(angle_ + gain_angle * innovation);
  bias_ += gain_bias * innovation;

  const double old_p00 = p00_;
  const double old_p01 = p01_;
  p00_ -= gain_angle * old_p00;
  p01_ -= gain_angle * old_p01;
  p10_ -= gain_bias * old_p00;
  p11_ -= gain_bias * old_p01;
  return angle_;
}

void ImuKalman::ScalarFilter::configure(
  double process_variance, double measurement_variance)
{
  process_variance_ = process_variance;
  measurement_variance_ = measurement_variance;
}

void ImuKalman::ScalarFilter::reset(double value)
{
  value_ = value;
  covariance_ = measurement_variance_;
}

double ImuKalman::ScalarFilter::update(double measurement, double dt)
{
  covariance_ += process_variance_ * dt;
  const double gain = covariance_ / (covariance_ + measurement_variance_);
  value_ += gain * (measurement - value_);
  covariance_ *= (1.0 - gain);
  return value_;
}

void ImuKalman::reset()
{
  calibrated_ = false;
  calibration_count_ = 0U;
  calibration_gyro_sum_ = {};
  calibration_accel_sum_ = {};
  gyro_bias_ = {};
  have_last_timestamp_ = false;
  last_timestamp_seconds_ = 0.0;
  yaw_ = 0.0;
  roll_filter_.reset(0.0, 0.0);
  pitch_filter_.reset(0.0, 0.0);
}

double ImuKalman::normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool ImuKalman::finite_vector(const Vector3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double ImuKalman::norm(const Vector3 & value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vector3 ImuKalman::map_vector(const Vector3 & input) const
{
  const Vector3 mapped{
    config_.axis_sign[0] * component(input, config_.axis_map[0]),
    config_.axis_sign[1] * component(input, config_.axis_map[1]),
    config_.axis_sign[2] * component(input, config_.axis_map[2]),
  };
  const double roll = config_.sensor_to_body_rpy_rad[0];
  const double pitch = config_.sensor_to_body_rpy_rad[1];
  const double yaw = config_.sensor_to_body_rpy_rad[2];
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  return {
    cy * cp * mapped.x + (cy * sp * sr - sy * cr) * mapped.y +
      (cy * sp * cr + sy * sr) * mapped.z,
    sy * cp * mapped.x + (sy * sp * sr + cy * cr) * mapped.y +
      (sy * sp * cr - cy * sr) * mapped.z,
    -sp * mapped.x + cp * sr * mapped.y + cp * cr * mapped.z,
  };
}

Vector3 ImuKalman::map_acceleration(const Vector3 & input) const
{
  return map_vector({
    input.x * config_.accel_scale[0],
    input.y * config_.accel_scale[1],
    input.z * config_.accel_scale[2],
  });
}

bool ImuKalman::stationary(
  const Vector3 & acceleration, const Vector3 & angular_velocity) const
{
  return norm(angular_velocity) <= config_.stationary_gyro_threshold &&
         std::abs(norm(acceleration) - config_.gravity) <=
         config_.stationary_accel_threshold;
}

UpdateResult ImuKalman::update(const ImuSample & sample)
{
  UpdateResult result;
  if (!finite_vector(sample.linear_acceleration) ||
    !finite_vector(sample.angular_velocity) ||
    !std::isfinite(sample.timestamp_seconds))
  {
    result.status = UpdateStatus::kInvalidSample;
    return result;
  }

  const Vector3 acceleration = map_acceleration(sample.linear_acceleration);
  const Vector3 angular_velocity = map_vector(sample.angular_velocity);

  double dt = 0.0;
  if (have_last_timestamp_) {
    dt = sample.timestamp_seconds - last_timestamp_seconds_;
    last_timestamp_seconds_ = sample.timestamp_seconds;
    if (dt < config_.minimum_dt || dt > config_.maximum_dt) {
      if (!calibrated_) {
        calibration_count_ = 0U;
        calibration_gyro_sum_ = {};
        calibration_accel_sum_ = {};
      }
      result.status = UpdateStatus::kInvalidDeltaTime;
      return result;
    }
  } else {
    have_last_timestamp_ = true;
    last_timestamp_seconds_ = sample.timestamp_seconds;
  }

  if (!calibrated_) {
    if (!stationary(acceleration, angular_velocity)) {
      calibration_count_ = 0U;
      calibration_gyro_sum_ = {};
      calibration_accel_sum_ = {};
      result.status = UpdateStatus::kCalibrationMotion;
      return result;
    }

    calibration_gyro_sum_.x += angular_velocity.x;
    calibration_gyro_sum_.y += angular_velocity.y;
    calibration_gyro_sum_.z += angular_velocity.z;
    calibration_accel_sum_.x += acceleration.x;
    calibration_accel_sum_.y += acceleration.y;
    calibration_accel_sum_.z += acceleration.z;
    ++calibration_count_;
    if (calibration_count_ < config_.calibration_samples) {
      result.status = UpdateStatus::kCalibrating;
      return result;
    }

    const double inverse_count = 1.0 / static_cast<double>(calibration_count_);
    gyro_bias_ = {
      calibration_gyro_sum_.x * inverse_count,
      calibration_gyro_sum_.y * inverse_count,
      calibration_gyro_sum_.z * inverse_count,
    };
    const Vector3 mean_acceleration{
      calibration_accel_sum_.x * inverse_count,
      calibration_accel_sum_.y * inverse_count,
      calibration_accel_sum_.z * inverse_count,
    };
    const double initial_roll = std::atan2(mean_acceleration.y, mean_acceleration.z);
    const double initial_pitch = std::atan2(
      -mean_acceleration.x,
      std::hypot(mean_acceleration.y, mean_acceleration.z));
    roll_filter_.reset(initial_roll, gyro_bias_.x);
    pitch_filter_.reset(initial_pitch, gyro_bias_.y);
    accel_filters_[0].reset(acceleration.x);
    accel_filters_[1].reset(acceleration.y);
    accel_filters_[2].reset(acceleration.z);
    gyro_filters_[0].reset(angular_velocity.x - gyro_bias_.x);
    gyro_filters_[1].reset(angular_velocity.y - gyro_bias_.y);
    gyro_filters_[2].reset(angular_velocity.z - gyro_bias_.z);
    calibrated_ = true;
    result.status = UpdateStatus::kReady;
    result.estimate = current_estimate(acceleration, angular_velocity, 0.0);
    return result;
  }

  const double measured_roll = std::atan2(acceleration.y, acceleration.z);
  const double measured_pitch = std::atan2(
    -acceleration.x, std::hypot(acceleration.y, acceleration.z));
  const double gravity_error = std::abs(norm(acceleration) - config_.gravity);
  const double normalized_error = gravity_error /
    std::max(config_.adaptive_accel_threshold, 1e-9);
  // |a|-g alone cannot identify horizontal body acceleration: a 1 m/s^2
  // fore-aft acceleration changes the norm by only about 0.05 m/s^2, yet it
  // looks like almost six degrees of pitch to an accelerometer tilt estimate.
  // Compare the measured specific-force vector with gravity predicted by the
  // gyro-propagated attitude. Real rotation is already present in the gyro
  // prediction; translation appears as a residual and must make the filter
  // trust the gyro more for this sample.
  const double predicted_roll = roll_filter_.predicted_angle(angular_velocity.x, dt);
  const double predicted_pitch = pitch_filter_.predicted_angle(angular_velocity.y, dt);
  const Vector3 predicted_gravity{
    -config_.gravity * std::sin(predicted_pitch),
    config_.gravity * std::sin(predicted_roll) * std::cos(predicted_pitch),
    config_.gravity * std::cos(predicted_roll) * std::cos(predicted_pitch),
  };
  const Vector3 gravity_residual{
    acceleration.x - predicted_gravity.x,
    acceleration.y - predicted_gravity.y,
    acceleration.z - predicted_gravity.z,
  };
  const double normalized_gravity_residual = norm(gravity_residual) /
    std::max(config_.adaptive_gravity_residual_threshold, 1e-9);
  const double measurement_scale = std::clamp(
    1.0 + config_.adaptive_accel_gain * normalized_error * normalized_error +
    config_.adaptive_gravity_residual_gain * normalized_gravity_residual *
    normalized_gravity_residual,
    1.0, config_.maximum_measurement_scale);

  roll_filter_.update(measured_roll, angular_velocity.x, dt, measurement_scale);
  pitch_filter_.update(measured_pitch, angular_velocity.y, dt, measurement_scale);
  yaw_ = normalize_angle(yaw_ + (angular_velocity.z - gyro_bias_.z) * dt);

  result.status = UpdateStatus::kReady;
  result.estimate = current_estimate(acceleration, angular_velocity, dt);
  return result;
}

Estimate ImuKalman::current_estimate(
  const Vector3 & acceleration, const Vector3 & angular_velocity, double dt)
{
  const double filter_dt = std::max(dt, config_.minimum_dt);
  const Vector3 corrected_gyro{
    angular_velocity.x - roll_filter_.bias(),
    angular_velocity.y - pitch_filter_.bias(),
    angular_velocity.z - gyro_bias_.z,
  };

  Estimate estimate;
  estimate.roll = roll_filter_.angle();
  estimate.pitch = pitch_filter_.angle();
  estimate.yaw = yaw_;
  estimate.linear_acceleration = {
    accel_filters_[0].update(acceleration.x, filter_dt),
    accel_filters_[1].update(acceleration.y, filter_dt),
    accel_filters_[2].update(acceleration.z, filter_dt),
  };
  estimate.angular_velocity = {
    gyro_filters_[0].update(corrected_gyro.x, filter_dt),
    gyro_filters_[1].update(corrected_gyro.y, filter_dt),
    gyro_filters_[2].update(corrected_gyro.z, filter_dt),
  };
  estimate.roll_variance = std::max(roll_filter_.angle_variance(), 1e-9);
  estimate.pitch_variance = std::max(pitch_filter_.angle_variance(), 1e-9);
  gyro_bias_.x = roll_filter_.bias();
  gyro_bias_.y = pitch_filter_.bias();
  return estimate;
}

}  // namespace imu_kalman_filter
