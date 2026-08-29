#ifndef IMU_KALMAN_FILTER__IMU_KALMAN_HPP_
#define IMU_KALMAN_FILTER__IMU_KALMAN_HPP_

#include <array>
#include <cstddef>

namespace imu_kalman_filter
{

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ImuSample
{
  Vector3 linear_acceleration;
  Vector3 angular_velocity;
  double timestamp_seconds{0.0};
};

struct FilterConfig
{
  std::array<int, 3> axis_map{{0, 1, 2}};
  std::array<double, 3> axis_sign{{1.0, 1.0, 1.0}};
  // Multiplicative sensor-axis accelerometer calibration, applied before
  // axis mapping and the fixed sensor-to-body rotation. It never affects gyro.
  std::array<double, 3> accel_scale{{1.0, 1.0, 1.0}};
  // Fixed proper rotation from the axis-mapped sensor frame into body frame.
  // RPY uses Rz(yaw) * Ry(pitch) * Rx(roll).
  std::array<double, 3> sensor_to_body_rpy_rad{{0.0, 0.0, 0.0}};

  std::size_t calibration_samples{200U};
  double gravity{9.80665};
  double stationary_gyro_threshold{0.15};
  double stationary_accel_threshold{0.8};
  double minimum_dt{0.001};
  double maximum_dt{0.1};

  double angle_process_variance{0.001};
  double bias_process_variance{0.003};
  double accel_angle_measurement_variance{0.03};
  double adaptive_accel_threshold{0.5};
  double adaptive_accel_gain{4.0};
  double adaptive_gravity_residual_threshold{0.35};
  double adaptive_gravity_residual_gain{8.0};
  double maximum_measurement_scale{100.0};

  double accel_process_variance{0.05};
  double accel_measurement_variance{0.25};
  double gyro_process_variance{0.001};
  double gyro_measurement_variance{0.01};
};

enum class UpdateStatus
{
  kCalibrating,
  kCalibrationMotion,
  kReady,
  kInvalidSample,
  kInvalidDeltaTime,
};

struct Estimate
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  Vector3 linear_acceleration;
  Vector3 angular_velocity;
  double roll_variance{0.0};
  double pitch_variance{0.0};
};

struct UpdateResult
{
  UpdateStatus status{UpdateStatus::kCalibrating};
  Estimate estimate;
};

/* Roll/pitch estimator for a six-axis IMU. It performs consecutive stationary
 * startup calibration, estimates gyro bias in the angle Kalman states, and
 * down-weights accelerometer tilt while |a| differs from gravity. Yaw is only
 * gyro-integrated and is intentionally not presented as observable. */
class ImuKalman
{
public:
  explicit ImuKalman(const FilterConfig & config = FilterConfig{});

  void reset();
  UpdateResult update(const ImuSample & sample);

  bool calibrated() const {return calibrated_;}
  std::size_t calibration_count() const {return calibration_count_;}
  const Vector3 & gyro_bias() const {return gyro_bias_;}

private:
  class AngleBiasFilter
  {
public:
    void configure(double q_angle, double q_bias, double r_measure);
    void reset(double angle, double bias);
    double update(
      double measured_angle, double gyro_rate, double dt,
      double measurement_scale);
    double angle() const {return angle_;}
    double bias() const {return bias_;}
    double predicted_angle(double gyro_rate, double dt) const
    {
      return ImuKalman::normalize_angle(angle_ + dt * (gyro_rate - bias_));
    }
    double angle_variance() const {return p00_;}

private:
    double q_angle_{0.001};
    double q_bias_{0.003};
    double r_measure_{0.03};
    double angle_{0.0};
    double bias_{0.0};
    double p00_{0.0};
    double p01_{0.0};
    double p10_{0.0};
    double p11_{0.0};
  };

  class ScalarFilter
  {
public:
    void configure(double process_variance, double measurement_variance);
    void reset(double value);
    double update(double measurement, double dt);

private:
    double process_variance_{0.01};
    double measurement_variance_{0.1};
    double value_{0.0};
    double covariance_{1.0};
  };

  static double normalize_angle(double angle);
  static bool finite_vector(const Vector3 & value);
  static double norm(const Vector3 & value);
  Vector3 map_vector(const Vector3 & input) const;
  Vector3 map_acceleration(const Vector3 & input) const;
  bool stationary(const Vector3 & acceleration, const Vector3 & angular_velocity) const;
  Estimate current_estimate(
    const Vector3 & acceleration, const Vector3 & angular_velocity,
    double dt);

  FilterConfig config_;
  bool calibrated_{false};
  std::size_t calibration_count_{0U};
  Vector3 calibration_gyro_sum_;
  Vector3 calibration_accel_sum_;
  Vector3 gyro_bias_;
  bool have_last_timestamp_{false};
  double last_timestamp_seconds_{0.0};
  double yaw_{0.0};

  AngleBiasFilter roll_filter_;
  AngleBiasFilter pitch_filter_;
  std::array<ScalarFilter, 3> accel_filters_;
  std::array<ScalarFilter, 3> gyro_filters_;
};

}  // namespace imu_kalman_filter

#endif  // IMU_KALMAN_FILTER__IMU_KALMAN_HPP_
