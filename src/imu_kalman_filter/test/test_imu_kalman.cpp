#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "imu_kalman_filter/imu_kalman.hpp"

#include "gtest/gtest.h"

namespace imu_kalman_filter
{
namespace
{
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

FilterConfig test_config()
{
  FilterConfig config;
  config.calibration_samples = 20U;
  return config;
}

UpdateResult feed(
  ImuKalman & filter, double & time, const Vector3 & acceleration,
  const Vector3 & angular_velocity)
{
  const UpdateResult result = filter.update({acceleration, angular_velocity, time});
  time += 0.01;
  return result;
}

void calibrate_level(ImuKalman & filter, double & time, const Vector3 & gyro_bias)
{
  for (std::size_t i = 0; i < 20U; ++i) {
    feed(filter, time, {0.0, 0.0, kGravity}, gyro_bias);
  }
  ASSERT_TRUE(filter.calibrated());
}

TEST(ImuKalmanTest, CalibratesGyroBiasFromConsecutiveStationarySamples)
{
  ImuKalman filter(test_config());
  double time = 0.0;
  const Vector3 bias{0.02, -0.01, 0.03};
  UpdateResult result;
  for (std::size_t i = 0; i < 20U; ++i) {
    result = feed(filter, time, {0.0, 0.0, kGravity}, bias);
  }

  ASSERT_EQ(result.status, UpdateStatus::kReady);
  EXPECT_NEAR(result.estimate.roll, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.pitch, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.angular_velocity.x, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.angular_velocity.y, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.angular_velocity.z, 0.0, 1e-9);
  EXPECT_NEAR(filter.gyro_bias().x, bias.x, 1e-9);
  EXPECT_NEAR(filter.gyro_bias().y, bias.y, 1e-9);
  EXPECT_NEAR(filter.gyro_bias().z, bias.z, 1e-9);
}

TEST(ImuKalmanTest, MotionRestartsStartupCalibration)
{
  ImuKalman filter(test_config());
  double time = 0.0;
  for (std::size_t i = 0; i < 10U; ++i) {
    feed(filter, time, {0.0, 0.0, kGravity}, {0.0, 0.0, 0.0});
  }
  EXPECT_EQ(filter.calibration_count(), 10U);

  const auto moving = feed(filter, time, {0.0, 0.0, kGravity}, {0.5, 0.0, 0.0});
  EXPECT_EQ(moving.status, UpdateStatus::kCalibrationMotion);
  EXPECT_EQ(filter.calibration_count(), 0U);
  EXPECT_FALSE(filter.calibrated());
}

TEST(ImuKalmanTest, DuplicateTimestampRestartsStartupCalibration)
{
  ImuKalman filter(test_config());
  const ImuSample stationary{{0.0, 0.0, kGravity}, {0.0, 0.0, 0.0}, 0.0};
  EXPECT_EQ(filter.update(stationary).status, UpdateStatus::kCalibrating);
  EXPECT_EQ(filter.calibration_count(), 1U);

  EXPECT_EQ(filter.update(stationary).status, UpdateStatus::kInvalidDeltaTime);
  EXPECT_EQ(filter.calibration_count(), 0U);
  EXPECT_FALSE(filter.calibrated());

  double time = 0.01;
  calibrate_level(filter, time, {0.0, 0.0, 0.0});
}

TEST(ImuKalmanTest, RejectsInvalidFrameOrNoiseConfiguration)
{
  FilterConfig left_handed = test_config();
  left_handed.axis_sign = {{1.0, 1.0, -1.0}};
  EXPECT_THROW(ImuKalman filter(left_handed), std::invalid_argument);

  FilterConfig non_finite = test_config();
  non_finite.adaptive_accel_gain = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ImuKalman filter(non_finite), std::invalid_argument);

  FilterConfig invalid_mounting = test_config();
  invalid_mounting.sensor_to_body_rpy_rad[1] = 2.0 * kPi;
  EXPECT_THROW(ImuKalman filter(invalid_mounting), std::invalid_argument);

  FilterConfig invalid_accel_scale = test_config();
  invalid_accel_scale.accel_scale[2] = 0.0;
  EXPECT_THROW(ImuKalman filter(invalid_accel_scale), std::invalid_argument);
}

TEST(ImuKalmanTest, AccelScaleIsAppliedBeforeCalibrationAndPublishing)
{
  constexpr double measured_gravity = 7.650638255779055;
  FilterConfig config = test_config();
  const double scale = kGravity / measured_gravity;
  config.accel_scale = {{scale, scale, scale}};
  ImuKalman filter(config);
  double time = 0.0;
  UpdateResult result;

  for (std::size_t i = 0; i < config.calibration_samples; ++i) {
    result = feed(
      filter, time, {0.0, 0.0, measured_gravity}, {0.01, -0.02, 0.03});
  }

  ASSERT_EQ(result.status, UpdateStatus::kReady);
  EXPECT_NEAR(result.estimate.roll, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.pitch, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.linear_acceleration.z, kGravity, 1e-9);
  EXPECT_NEAR(result.estimate.angular_velocity.z, 0.0, 1e-9);
}

TEST(ImuKalmanTest, SensorToBodyRotationRemovesFixedMountingTilt)
{
  const double mounting_roll = 1.239690290 * kPi / 180.0;
  const double mounting_pitch = -3.746790025 * kPi / 180.0;
  FilterConfig config = test_config();
  config.sensor_to_body_rpy_rad = {{mounting_roll, mounting_pitch, 0.0}};
  ImuKalman filter(config);
  double time = 0.0;

  // Gravity represented in the tilted sensor frame. The configured proper
  // rotation maps this back to [0, 0, g] in body coordinates.
  const Vector3 sensor_gravity{
    -kGravity * std::sin(mounting_pitch),
    kGravity * std::sin(mounting_roll) * std::cos(mounting_pitch),
    kGravity * std::cos(mounting_roll) * std::cos(mounting_pitch),
  };
  UpdateResult result;
  for (std::size_t i = 0; i < 20U; ++i) {
    result = feed(filter, time, sensor_gravity, {0.0, 0.0, 0.0});
  }

  ASSERT_EQ(result.status, UpdateStatus::kReady);
  EXPECT_NEAR(result.estimate.roll, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.pitch, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.linear_acceleration.x, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.linear_acceleration.y, 0.0, 1e-9);
  EXPECT_NEAR(result.estimate.linear_acceleration.z, kGravity, 1e-9);
}

TEST(ImuKalmanTest, TracksSlowRollWithGyroPredictionAndAccelCorrection)
{
  ImuKalman filter(test_config());
  double time = 0.0;
  calibrate_level(filter, time, {0.01, -0.02, 0.005});

  const double target_roll = 15.0 * kPi / 180.0;
  const double duration = 2.0;
  const double roll_rate = target_roll / duration;
  UpdateResult result;
  for (std::size_t i = 1; i <= 200U; ++i) {
    const double roll = target_roll * static_cast<double>(i) / 200.0;
    const Vector3 acceleration{
      0.0, kGravity * std::sin(roll), kGravity * std::cos(roll)};
    result = feed(filter, time, acceleration, {roll_rate + 0.01, -0.02, 0.005});
  }
  for (std::size_t i = 0; i < 300U; ++i) {
    const Vector3 acceleration{
      0.0, kGravity * std::sin(target_roll), kGravity * std::cos(target_roll)};
    result = feed(filter, time, acceleration, {0.01, -0.02, 0.005});
  }

  ASSERT_EQ(result.status, UpdateStatus::kReady);
  EXPECT_NEAR(result.estimate.roll, target_roll, 0.015);
  EXPECT_NEAR(result.estimate.pitch, 0.0, 0.01);
}

TEST(ImuKalmanTest, AdaptiveCovarianceLimitsFalseTiltDuringLinearAcceleration)
{
  FilterConfig config = test_config();
  config.adaptive_accel_gain = 20.0;
  ImuKalman filter(config);
  double time = 0.0;
  calibrate_level(filter, time, {0.0, 0.0, 0.0});

  UpdateResult result;
  for (std::size_t i = 0; i < 50U; ++i) {
    result = feed(filter, time, {8.0, 0.0, kGravity}, {0.0, 0.0, 0.0});
  }
  ASSERT_EQ(result.status, UpdateStatus::kReady);
  EXPECT_LT(std::abs(result.estimate.pitch), 0.12);
}

TEST(ImuKalmanTest, RejectsHorizontalAccelerationWhoseNormRemainsCloseToGravity)
{
  FilterConfig config = test_config();
  config.adaptive_gravity_residual_gain = 12.0;
  ImuKalman filter(config);
  double time = 0.0;
  calibrate_level(filter, time, {0.0, 0.0, 0.0});

  UpdateResult result;
  for (std::size_t i = 0; i < 100U; ++i) {
    // This changes |a| by only 0.05 m/s^2, so norm-only gating accepts it as
    // gravity and converges toward a false 5.8 degree pitch.
    result = feed(filter, time, {1.0, 0.0, kGravity}, {0.0, 0.0, 0.0});
  }
  ASSERT_EQ(result.status, UpdateStatus::kReady);
  EXPECT_LT(std::abs(result.estimate.pitch), 0.035);
}

TEST(ImuKalmanTest, GravityResidualGateStillTracksGyroConsistentRotation)
{
  ImuKalman filter(test_config());
  double time = 0.0;
  calibrate_level(filter, time, {0.0, 0.0, 0.0});

  const double target_roll = 10.0 * kPi / 180.0;
  const double roll_rate = target_roll;
  UpdateResult result;
  for (std::size_t i = 1; i <= 100U; ++i) {
    const double roll = target_roll * static_cast<double>(i) / 100.0;
    result = feed(
      filter, time,
      {0.0, kGravity * std::sin(roll), kGravity * std::cos(roll)},
      {roll_rate, 0.0, 0.0});
  }
  ASSERT_EQ(result.status, UpdateStatus::kReady);
  EXPECT_NEAR(result.estimate.roll, target_roll, 0.02);
}

TEST(ImuKalmanTest, ScalarKalmanOutputReducesAlternatingSensorNoise)
{
  ImuKalman filter(test_config());
  double time = 0.0;
  calibrate_level(filter, time, {0.0, 0.0, 0.0});

  double raw_absolute_sum = 0.0;
  double filtered_absolute_sum = 0.0;
  for (std::size_t i = 0; i < 400U; ++i) {
    const double noise = (i % 2U == 0U) ? 0.5 : -0.5;
    const auto result = feed(
      filter, time, {noise, 0.0, kGravity}, {noise * 0.05, 0.0, 0.0});
    ASSERT_EQ(result.status, UpdateStatus::kReady);
    raw_absolute_sum += std::abs(noise);
    filtered_absolute_sum += std::abs(result.estimate.linear_acceleration.x);
  }
  EXPECT_LT(filtered_absolute_sum, raw_absolute_sum * 0.4);
}

TEST(ImuKalmanTest, RejectsDuplicateOrLongDeltaTime)
{
  ImuKalman filter(test_config());
  double time = 0.0;
  calibrate_level(filter, time, {0.0, 0.0, 0.0});

  const ImuSample duplicate{{0.0, 0.0, kGravity}, {0.0, 0.0, 0.0}, time - 0.01};
  EXPECT_EQ(filter.update(duplicate).status, UpdateStatus::kInvalidDeltaTime);

  const ImuSample late{{0.0, 0.0, kGravity}, {0.0, 0.0, 0.0}, time + 1.0};
  EXPECT_EQ(filter.update(late).status, UpdateStatus::kInvalidDeltaTime);
}

}  // namespace
}  // namespace imu_kalman_filter
