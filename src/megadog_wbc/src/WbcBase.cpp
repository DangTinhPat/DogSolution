// Ported from skywoodsz/qm_control's qm_wbc::WbcBase.cpp - see
// include/megadog_wbc/WbcBase.h for what was stripped/replaced.

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "megadog_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace megadog
{
namespace hwbc
{
using namespace ocs2;
using namespace ocs2::legged_robot;

WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                 const PinocchioEndEffectorKinematics& eeKinematics, HierarchicalWbcConfig config)
    : config_(config),
      pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      eeKinematics_(eeKinematics.clone())
{
    numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts;  // \dot v, f
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    qDesired_ = vector_t(info_.generalizedCoordinatesNum);
    vDesired_ = vector_t(info_.generalizedCoordinatesNum);
    inputLast_ = vector_t::Zero(info_.inputDim);
    baseAccDesired_ = vector_t(6);

    base_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    base_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);

    // Per-joint effort limit, assumed identical across all 4 legs (true for
    // babyDog's URDF: every leg joint is rated 9.1 N.m). Loaded once here
    // rather than from a task file - see HierarchicalWbcConfig for the
    // tunable gains, which are the only per-deployment-varying settings.
    legTorqueLimits_ = pinocchioInterfaceMeasured_.getModel().effortLimit.segment<3>(6);
    if (config_.leg_torque_limits_nm.size() == 3 &&
        std::all_of(config_.leg_torque_limits_nm.begin(), config_.leg_torque_limits_nm.end(),
                    [](const double value) { return std::isfinite(value) && value > 0.0; })) {
        for (int joint = 0; joint < 3; ++joint) {
            legTorqueLimits_(joint) = config_.leg_torque_limits_nm[static_cast<std::size_t>(joint)];
        }
    } else if (std::isfinite(config_.torque_limit_scale) && config_.torque_limit_scale > 0.0) {
        legTorqueLimits_ *= config_.torque_limit_scale;
    }

    // Unlike torque limits, abad position limits mirror sign per leg side
    // (see CLAUDE.md's "home = 0" invariant), so all 12 are read directly -
    // no single-leg-replicated shortcut here.
    jointLowerLimits_ = pinocchioInterfaceMeasured_.getModel().lowerPositionLimit.tail(info_.actuatedDofNum);
    jointUpperLimits_ = pinocchioInterfaceMeasured_.getModel().upperPositionLimit.tail(info_.actuatedDofNum);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                         size_t mode, scalar_t period, scalar_t /*time*/)
{
    contactFlag_ = modeNumber2StanceLeg(mode);
    numContacts_ = 0;
    for (bool flag : contactFlag_) {
        if (flag) {
            numContacts_++;
        }
    }

    updateMeasured(rbdStateMeasured);
    updateDesired(stateDesired, inputDesired, period);

    return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured)
{
    qMeasured_.setZero();
    vMeasured_.setZero();

    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
        qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
    vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

    const auto& model = pinocchioInterfaceMeasured_.getModel();
    auto& data = pinocchioInterfaceMeasured_.getData();

    // For floating base EoM task
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, qMeasured_);

    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // For floating base EoM task
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    // For not-contact-motion task
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    // For base motion tracking task
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> base_j, base_dj;
    base_j.setZero(6, info_.generalizedCoordinatesNum);
    base_dj.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_j);
    pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_dj);
    base_j_ = base_j;
    base_dj_ = base_dj;
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
{
    const auto& model = pinocchioInterfaceDesired_.getModel();
    auto& data = pinocchioInterfaceDesired_.getData();

    qDesired_.setZero();
    vDesired_.setZero();

    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
    qDesired_ = mapping_.getPinocchioJointPosition(stateDesired);
    pinocchio::forwardKinematics(model, data, qDesired_);
    pinocchio::computeJointJacobians(model, data, qDesired_);
    pinocchio::updateFramePlacements(model, data);
    updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired_);

    vDesired_ = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);

    // update base acc desired
    jointAccel_ = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    inputLast_ = inputDesired;

    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
    using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;

    const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    const Matrix6 Ab = A.template leftCols<6>();
    const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
    auto Aj = A.rightCols(info_.actuatedDofNum);
    const auto ADot = pinocchio::dccrba(model, data, qDesired_, vDesired_);
    Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    centroidalMomentumRate.noalias() -= ADot * vDesired_;
    centroidalMomentumRate.noalias() -= Aj * jointAccel_;

    baseAccDesired_.setZero();
    baseAccDesired_ = AbInv * centroidalMomentumRate;
}

Task WbcBase::formulateBaseLinearMotionTask()
{
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);

    b = baseAccDesired_.head(2) + config_.base_linear_kp * (qDesired_.head(2) - qMeasured_.head(2)) +
        config_.base_linear_kd * (vDesired_.head(2) - vMeasured_.head(2));

    return {a, b, matrix_t(), vector_t()};
}

// Tracking base angular motion task
Task WbcBase::formulateBaseAngularMotionTask()
{
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

    vector3_t eulerAngles = qMeasured_.segment<3>(3);

    // from derivative euler to angular
    vector3_t vMeasuredGlobal =
        getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
    vector3_t vDesiredGlobal =
        getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vDesired_.segment<3>(3));

    // from euler to rotation
    vector3_t eulerAnglesDesired;
    eulerAnglesDesired << qDesired_.segment<3>(3);
    matrix3_t rotationBaseMeasuredToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
    matrix3_t rotationBaseReferenceToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

    vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);

    // desired acc
    vector3_t accDesired = getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(
        eulerAngles, vDesired_.segment<3>(3), baseAccDesired_.segment<3>(3));

    b = accDesired + config_.base_angular_kp * error + config_.base_angular_kd * (vDesiredGlobal - vMeasuredGlobal) -
        base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

// Tracking base height motion task
Task WbcBase::formulateBaseHeightMotionTask()
{
    matrix_t a(1, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 2, 1, 1) = matrix_t::Identity(1, 1);

    b[0] = baseAccDesired_[2] + config_.base_height_kp * (qDesired_[2] - qMeasured_[2]) +
           config_.base_height_kd * (vDesired_[2] - vMeasured_[2]);

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateHaaJointPostureTask()
{
    const size_t numHaaJoints = info_.actuatedDofNum / 3;
    matrix_t a(numHaaJoints, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    const vector_t qJointMeasured = qMeasured_.tail(info_.actuatedDofNum);
    const vector_t vJointMeasured = vMeasured_.tail(info_.actuatedDofNum);
    const vector_t qJointDesired = qDesired_.tail(info_.actuatedDofNum);
    const vector_t vJointDesired = vDesired_.tail(info_.actuatedDofNum);

    size_t row = 0;
    for (size_t joint = 0; joint < info_.actuatedDofNum; joint += 3) {
        const bool useConfiguredNominal = config_.haa_posture_nominal_rad.size() == numHaaJoints &&
                                          std::isfinite(config_.haa_posture_nominal_rad[row]);
        const scalar_t qTarget = useConfiguredNominal
            ? static_cast<scalar_t>(config_.haa_posture_nominal_rad[row])
            : qJointDesired(static_cast<long>(joint));
        a(static_cast<long>(row), static_cast<long>(6 + joint)) = 1.0;
        b(static_cast<long>(row)) =
            config_.haa_posture_kp * (qTarget - qJointMeasured(static_cast<long>(joint))) +
            config_.haa_posture_kd * (vJointDesired(static_cast<long>(joint)) - vJointMeasured(static_cast<long>(joint)));
        ++row;
    }

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateLegJointPostureTask()
{
    matrix_t a(info_.actuatedDofNum, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    const vector_t qJointMeasured = qMeasured_.tail(info_.actuatedDofNum);
    const vector_t vJointMeasured = vMeasured_.tail(info_.actuatedDofNum);
    const vector_t qJointDesired = qDesired_.tail(info_.actuatedDofNum);
    const vector_t vJointDesired = vDesired_.tail(info_.actuatedDofNum);

    for (size_t joint = 0; joint < info_.actuatedDofNum; ++joint) {
        const bool useConfiguredNominal = config_.leg_posture_nominal_rad.size() == info_.actuatedDofNum &&
                                          std::isfinite(config_.leg_posture_nominal_rad[joint]);
        const scalar_t qTarget = useConfiguredNominal
            ? static_cast<scalar_t>(config_.leg_posture_nominal_rad[joint])
            : qJointDesired(static_cast<long>(joint));
        a(static_cast<long>(joint), static_cast<long>(6 + joint)) = 1.0;
        b(static_cast<long>(joint)) =
            config_.leg_posture_kp * (qTarget - qJointMeasured(static_cast<long>(joint))) +
            config_.leg_posture_kd * (vJointDesired(static_cast<long>(joint)) - vJointMeasured(static_cast<long>(joint)));
    }

    return {a, b, matrix_t(), vector_t()};
}

// [J, 0] x = \dot V - \dotJ v
Task WbcBase::formulateSwingLegTask()
{
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        if (!contactFlag_[i]) {
            vector3_t accel = config_.swing_kp * (posDesired[i] - posMeasured[i]) + config_.swing_kd * (velDesired[i] - velMeasured[i]);
            a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
            b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
            j++;
        }
    }

    return {a, b, matrix_t(), vector_t()};
}

// EoM
// [Mb, -J^Tb]x = -hb
Task WbcBase::formulateFloatingBaseEomTask()
{
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mb, Jb_T;
    vector_t hb;
    Mb = data.M.topRows(6);
    hb = data.nle.topRows(6);
    Jb_T = j_.transpose().topRows(6);

    a << Mb, -Jb_T;
    b = -hb;

    return {a, b, matrix_t(), vector_t()};
}

// torque limit
// tau_min - hj <= [Mj, -Jj^T] <= tau_max - hj
Task WbcBase::formulateTorqueLimitsTask()
{
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    vector_t f(d.rows());
    d.setZero();
    f.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);

    d.block(0, 0, info_.actuatedDofNum, numDecisionVars_) << Mj, -Jj_T;
    d.block(info_.actuatedDofNum, 0, info_.actuatedDofNum, numDecisionVars_) << -Mj, Jj_T;

    // babyDog has 4 legs of 3 joints each and no arm. Build the 12-joint
    // limit vector once, then apply it to both sides of the inequality:
    //   tau <= limit  ->  [Mj, -Jj^T] x <=  limit - hj
    //  -tau <= limit  -> [-Mj,  Jj^T] x <=  limit + hj
    vector_t jointTorqueLimits(info_.actuatedDofNum);
    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
        jointTorqueLimits.segment<3>(3 * leg) = legTorqueLimits_;
    }
    f.segment(0, info_.actuatedDofNum) = jointTorqueLimits - hj;
    f.segment(info_.actuatedDofNum, info_.actuatedDofNum) = jointTorqueLimits + hj;

    return {matrix_t(), vector_t(), d, f};
}

// [J, 0] x = -\dot J v
Task WbcBase::formulateNoContactMotionTask()
{
    matrix_t a(3 * numContacts_, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
        if (contactFlag_[i]) {
            a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
            b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
            j++;
        }
    }

    return {a, b, matrix_t(), vector_t()};
}

// no contact:
// [0, I] x = 0
// contact:
// [0, C] x <= 0
Task WbcBase::formulateFrictionConeTask()
{
    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    a.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        if (!contactFlag_[i]) {
            a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }
    }
    vector_t b(a.rows());
    b.setZero();

    matrix_t frictionPyramid(5, 3);
    frictionPyramid << 0, 0, -1, 1, 0, -config_.friction_coefficient, -1, 0, -config_.friction_coefficient, 0, 1,
        -config_.friction_coefficient, 0, -1, -config_.friction_coefficient;

    matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    d.setZero();
    j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        if (contactFlag_[i]) {
            d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramid;
        }
    }
    vector_t f = vector_t::Zero(d.rows());

    return {a, b, d, f};
}

// Joint position-limit avoidance: bound each joint's optimized acceleration
// so that extrapolating current position/velocity forward by a fixed
// horizon would not cross the URDF position limit, i.e. treat the joint as
// a double integrator and require
//   q + v*T + 0.5*qddot*T^2 <= q_max   =>   qddot <= 2*(q_max - q - v*T) / T^2
//   q + v*T + 0.5*qddot*T^2 >= q_min   =>   qddot >= 2*(q_min - q - v*T) / T^2
// This is the same joint block of the decision vector as the torque-limit
// task ([0, I, 0] selecting qddot_joint), just bounding acceleration instead
// of the resulting torque. See HierarchicalWbcConfig::joint_limit_horizon_seconds.
Task WbcBase::formulateJointLimitsTask()
{
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    vector_t f(d.rows());
    d.setZero();
    f.setZero();

    d.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum) =
        matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
    d.block(info_.actuatedDofNum, 6, info_.actuatedDofNum, info_.actuatedDofNum) =
        -matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

    const scalar_t horizon = std::max(config_.joint_limit_horizon_seconds, 1e-3);
    const scalar_t horizonSq = horizon * horizon;

    vector_t qJoint = qMeasured_.tail(info_.actuatedDofNum);
    vector_t vJoint = vMeasured_.tail(info_.actuatedDofNum);

    vector_t qddotMax = (2.0 / horizonSq) * (jointUpperLimits_ - qJoint - vJoint * horizon);
    vector_t qddotMin = (2.0 / horizonSq) * (jointLowerLimits_ - qJoint - vJoint * horizon);

    f.segment(0, info_.actuatedDofNum) = qddotMax;
    f.segment(info_.actuatedDofNum, info_.actuatedDofNum) = -qddotMin;

    return {matrix_t(), vector_t(), d, f};
}

// [0, I] x = GRFs
Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const
{
    matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
    b = inputDesired.head(a.rows());

    return {a, b, matrix_t(), vector_t()};
}

vector_t WbcBase::updateCmd(vector_t x_optimal)
{
    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);
    matrix_t a = (matrix_t(info_.actuatedDofNum, getNumDecisionVars()) << Mj, -Jj_T).finished();

    vector_t torque_optimal = a * x_optimal + hj;

    vector_t cmd = (vector_t(numDecisionVars_ + info_.actuatedDofNum) << x_optimal, torque_optimal).finished();

    return cmd;
}

}  // namespace hwbc
}  // namespace megadog
