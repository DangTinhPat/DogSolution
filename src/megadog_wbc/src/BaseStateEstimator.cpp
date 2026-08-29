#include "megadog_wbc/BaseStateEstimator.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <cmath>
#include <utility>

namespace megadog
{
namespace hwbc
{
using namespace ocs2;
using namespace ocs2::legged_robot;

namespace
{
// Same defaults as legged_estimation::KalmanFilterEstimate's task.info
// "kalmanFilter" section (qiayuanl/legged_control) - a literature-standard
// MIT-Cheetah-style leg-odometry estimator, untuned starting point for
// megaDog's own mass/leg-length scale.
constexpr double kFootRadius = 0.02;
constexpr double kImuProcessNoisePosition = 0.02;
constexpr double kImuProcessNoiseVelocity = 0.02;
constexpr double kFootProcessNoisePosition = 0.002;
constexpr double kFootSensorNoisePosition = 0.005;
constexpr double kFootSensorNoiseVelocity = 0.1;
constexpr double kFootHeightSensorNoise = 0.01;
// Inflates a swing foot's process/measurement noise so the filter
// effectively ignores it - same "high_suspect_number" qiayuanl's original
// uses, not a physically-derived constant.
constexpr double kHighSuspectNumber = 100.0;
}  // namespace

BaseStateEstimator::BaseStateEstimator(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                       const PinocchioEndEffectorKinematics& eeKinematics, double initial_height_m)
    : pinocchioInterface_(pinocchioInterface),
      info_(std::move(info)),
      eeKinematics_(eeKinematics.clone()),
      numContacts_(info_.numThreeDofContacts),
      dimContacts_(3 * numContacts_),
      numState_(6 + dimContacts_),
      numObserve_(2 * dimContacts_ + numContacts_)
{
    eeKinematics_->setPinocchioInterface(pinocchioInterface_);

    a_.setIdentity(static_cast<Eigen::Index>(numState_), static_cast<Eigen::Index>(numState_));
    b_.setZero(static_cast<Eigen::Index>(numState_), 3);
    matrix_t c1(3, 6), c2(3, 6);
    c1 << matrix3_t::Identity(), matrix3_t::Zero();
    c2 << matrix3_t::Zero(), matrix3_t::Identity();
    c_.setZero(static_cast<Eigen::Index>(numObserve_), static_cast<Eigen::Index>(numState_));
    for (size_t i = 0; i < numContacts_; ++i) {
        const auto row1 = static_cast<Eigen::Index>(3 * i);
        const auto row2 = static_cast<Eigen::Index>(3 * (numContacts_ + i));
        c_.block(row1, 0, 3, 6) = c1;
        c_.block(row2, 0, 3, 6) = c2;
        c_(static_cast<Eigen::Index>(2 * dimContacts_ + i), static_cast<Eigen::Index>(6 + 3 * i + 2)) = 1.0;
    }
    c_.block(0, 6, static_cast<Eigen::Index>(dimContacts_), static_cast<Eigen::Index>(dimContacts_)) =
        -matrix_t::Identity(static_cast<Eigen::Index>(dimContacts_), static_cast<Eigen::Index>(dimContacts_));

    reset(initial_height_m);
}

void BaseStateEstimator::reset(double initial_height_m)
{
    xHat_.setZero(static_cast<Eigen::Index>(numState_));
    xHat_(2) = initial_height_m;
    // Foot world-position state starts at ground level (z=0), matching the
    // flat-ground assumption update() otherwise enforces every tick via the
    // (always-zero) feetHeights measurement.
    p_ = 100.0 * matrix_t::Identity(static_cast<Eigen::Index>(numState_), static_cast<Eigen::Index>(numState_));
}

BaseStateEstimator::Output BaseStateEstimator::update(const Input& input)
{
    Output output;
    const double dt = input.dt_s;
    if (!std::isfinite(dt) || dt <= 0.0) {
        output.base_pos_m = {xHat_(0), xHat_(1), xHat_(2)};
        output.base_linear_vel_m_s = {xHat_(3), xHat_(4), xHat_(5)};
        return output;
    }

    a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();
    b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
    b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();

    const auto dimContactsIdx = static_cast<Eigen::Index>(dimContacts_);
    const auto numContactsIdx = static_cast<Eigen::Index>(numContacts_);

    matrix_t q = matrix_t::Identity(static_cast<Eigen::Index>(numState_), static_cast<Eigen::Index>(numState_));
    q.block(0, 0, 3, 3) = (dt / 20.0) * kImuProcessNoisePosition * matrix3_t::Identity();
    q.block(3, 3, 3, 3) = (dt * 9.81 / 20.0) * kImuProcessNoiseVelocity * matrix3_t::Identity();
    q.block(6, 6, dimContactsIdx, dimContactsIdx) = dt * kFootProcessNoisePosition * matrix_t::Identity(dimContactsIdx, dimContactsIdx);

    matrix_t r = matrix_t::Identity(static_cast<Eigen::Index>(numObserve_), static_cast<Eigen::Index>(numObserve_));
    r.block(0, 0, dimContactsIdx, dimContactsIdx) *= kFootSensorNoisePosition;
    r.block(dimContactsIdx, dimContactsIdx, dimContactsIdx, dimContactsIdx) *= kFootSensorNoiseVelocity;
    r.block(2 * dimContactsIdx, 2 * dimContactsIdx, numContactsIdx, numContactsIdx) *= kFootHeightSensorNoise;

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();

    // Base held at the world origin, orientation from IMU - forward
    // kinematics then gives each foot's position/velocity *relative to the
    // base*, exactly what the leg-odometry measurement model needs (see this
    // file's header comment).
    vector_t qPino = vector_t::Zero(info_.generalizedCoordinatesNum);
    qPino(3) = input.base_euler_zyx_rad[0];
    qPino(4) = input.base_euler_zyx_rad[1];
    qPino(5) = input.base_euler_zyx_rad[2];
    for (size_t i = 0; i < info_.actuatedDofNum; ++i) {
        qPino(static_cast<Eigen::Index>(6 + i)) = input.joint_pos_rad[i];
    }

    vector_t vPino = vector_t::Zero(info_.generalizedCoordinatesNum);
    vPino(3) = input.base_euler_zyx_rate_rad_s[0];
    vPino(4) = input.base_euler_zyx_rate_rad_s[1];
    vPino(5) = input.base_euler_zyx_rate_rad_s[2];
    for (size_t i = 0; i < info_.actuatedDofNum; ++i) {
        vPino(static_cast<Eigen::Index>(6 + i)) = input.joint_vel_rad_s[i];
    }

    pinocchio::forwardKinematics(model, data, qPino, vPino);
    pinocchio::updateFramePlacements(model, data);

    const auto eePos = eeKinematics_->getPosition(vector_t());
    const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

    vector_t ps = vector_t::Zero(dimContactsIdx);
    vector_t vs = vector_t::Zero(dimContactsIdx);
    for (size_t i = 0; i < numContacts_; ++i) {
        const auto i3 = static_cast<Eigen::Index>(3 * i);
        const bool isContact = input.contact_flag[i];
        const double scale = isContact ? 1.0 : kHighSuspectNumber;

        q.block(6 + i3, 6 + i3, 3, 3) *= scale;
        r.block(i3, i3, 3, 3) *= scale;
        r.block(dimContactsIdx + i3, dimContactsIdx + i3, 3, 3) *= scale;
        r(2 * dimContactsIdx + static_cast<Eigen::Index>(i), 2 * dimContactsIdx + static_cast<Eigen::Index>(i)) *= scale;

        ps.segment(i3, 3) = -eePos[i];
        ps(i3 + 2) += kFootRadius;
        vs.segment(i3, 3) = -eeVel[i];
    }

    const vector3_t eulerZyx(input.base_euler_zyx_rad[0], input.base_euler_zyx_rad[1], input.base_euler_zyx_rad[2]);
    const matrix3_t rotationWorldFromBody = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerZyx);
    const vector3_t accelLocal(input.base_linear_accel_local_m_s2[0], input.base_linear_accel_local_m_s2[1],
                               input.base_linear_accel_local_m_s2[2]);
    // IMU accelerometers read specific force (true accel minus gravity), so
    // rotating that into world frame and adding gravity back recovers true
    // world-frame acceleration - a stationary, level IMU reads (0,0,+9.81),
    // giving accelWorld = 0 here as expected.
    const vector3_t gravity(0.0, 0.0, -9.81);
    const vector3_t accelWorld = rotationWorldFromBody * accelLocal + gravity;

    // Flat-ground assumption: a planted foot's world height is always 0
    // (megaDog has no external tracking-camera correction to feed this
    // otherwise, see this file's header comment).
    const vector_t feetHeights = vector_t::Zero(numContactsIdx);

    vector_t y(static_cast<Eigen::Index>(numObserve_));
    y << ps, vs, feetHeights;

    xHat_ = a_ * xHat_ + b_ * accelWorld;
    const matrix_t pm = a_ * p_ * a_.transpose() + q;
    const matrix_t cT = c_.transpose();
    const vector_t ey = y - c_ * xHat_;
    const matrix_t s = c_ * pm * cT + r;

    const vector_t sEy = s.lu().solve(ey);
    xHat_ += pm * cT * sEy;

    const matrix_t sC = s.lu().solve(c_);
    p_ = (matrix_t::Identity(static_cast<Eigen::Index>(numState_), static_cast<Eigen::Index>(numState_)) - pm * cT * sC) * pm;
    p_ = ((p_ + p_.transpose()) / 2.0).eval();

    output.base_pos_m = {xHat_(0), xHat_(1), xHat_(2)};
    output.base_linear_vel_m_s = {xHat_(3), xHat_(4), xHat_(5)};
    return output;
}

}  // namespace hwbc
}  // namespace megadog
