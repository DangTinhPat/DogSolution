#pragma once

// Base position/velocity estimator for the floating base - replaces the
// fixed local-origin/zero-velocity placeholder MegadogWbcRuntime previously
// fed the base_height/base_linear Cartesian tasks in WbcBase.cpp with (see
// MegadogWbcMeasurement's doc comment in MegadogWbcRuntime.h). Ported from
// qiayuanl/legged_control's legged_estimation::KalmanFilterEstimate (see
// src/megadog_legged_control/legged_estimation - not built, COLCON_IGNORE,
// ROS1 API) to plain OCS2 types with no ROS dependency: same 18-state linear
// KF (base position(3) + velocity(3) + one world-frame position(3) per foot),
// same leg-odometry measurement model - a planted foot's world position is
// assumed constant and its world velocity zero, so contact legs indirectly
// observe base position/velocity through forward kinematics (joint encoders
// + IMU orientation); a leg's contact flag scales its process/measurement
// noise between "trust it" and "ignore it" (see BaseStateEstimator.cpp).
// legged_estimation's external tracking-camera correction (updateFromTopic())
// has no equivalent here - megaDog has no such sensor - so foot height stays
// pinned at the flat-ground assumption (0) instead.
//
// No leg contact SENSOR exists on this robot (or in sim) - the contact flag
// fed in is MegadogWbcRuntime's own gait-schedule-predicted stance/swing mode
// (modeNumber2StanceLeg(plannedMode)), the same model-based contact estimate
// the rest of the WBC/MPC pipeline already relies on.

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <array>
#include <memory>

namespace megadog
{
namespace hwbc
{

class BaseStateEstimator
{
public:
    struct Input
    {
        std::array<double, 3> base_euler_zyx_rad{};
        std::array<double, 3> base_euler_zyx_rate_rad_s{};
        std::array<double, 3> base_linear_accel_local_m_s2{};
        std::array<double, 12> joint_pos_rad{};
        std::array<double, 12> joint_vel_rad_s{};
        ocs2::legged_robot::contact_flag_t contact_flag{};
        double dt_s = 0.0;
    };

    struct Output
    {
        std::array<double, 3> base_pos_m{0.0, 0.0, 0.0};
        std::array<double, 3> base_linear_vel_m_s{0.0, 0.0, 0.0};
    };

    BaseStateEstimator(const ocs2::PinocchioInterface& pinocchioInterface, ocs2::CentroidalModelInfo info,
                       const ocs2::PinocchioEndEffectorKinematics& eeKinematics, double initial_height_m);

    // Reinitializes position (at initial_height_m)/velocity/foot-position
    // state and covariance - the constructor already calls this once, so a
    // fresh MegadogWbcRuntime never needs a separate call; only meaningful if
    // an owner ever wants to re-seed a long-lived estimator mid-run.
    void reset(double initial_height_m);

    // Call once per control tick, same rate as MegadogWbcRuntime::update()
    // (dt_s <= 0 or non-finite: returns the last valid estimate unchanged).
    Output update(const Input& input);

private:
    ocs2::PinocchioInterface pinocchioInterface_;
    ocs2::CentroidalModelInfo info_;
    std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematics_;

    size_t numContacts_;
    size_t dimContacts_;
    size_t numState_;
    size_t numObserve_;

    // a_/b_/c_ are fixed in structure (only a_'s/b_'s dt-dependent blocks
    // change per tick); q_/r_ base noise levels are rebuilt fresh each tick
    // (self-contained in update(), not stored).
    ocs2::matrix_t a_, b_, c_;
    ocs2::matrix_t p_;
    ocs2::vector_t xHat_;
};

}  // namespace hwbc
}  // namespace megadog
