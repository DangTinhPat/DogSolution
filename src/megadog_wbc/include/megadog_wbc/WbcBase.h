#ifndef CONTROLLER_CONTROL_WBCBASE_H
#define CONTROLLER_CONTROL_WBCBASE_H

// Ported from skywoodsz/qm_control's qm_wbc::WbcBase, with every arm-specific
// task/member stripped (babyDog has no arm - see
// /home/dvt/.claude/plans/purrfect-imagining-hartmanis.md, Milestone 3) and
// ros::NodeHandle/dynamic_reconfigure replaced by a plain HierarchicalWbcConfig
// struct. Gains below are qm_control's literal dynamic_reconfigure defaults
// (cfg/wbcWigeht.cfg) carried over as an untuned starting point, same
// provenance note as babydog_legged_interface/config/task.info's Q/R weights.
//
// Decision vector: x = [v_dot^T, F^T]^T, size
// info.generalizedCoordinatesNum + 3*info.numThreeDofContacts (30 for
// babyDog: 6 floating-base + 12 joint accelerations, plus 12 contact forces).

#include "megadog_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <vector>

namespace megadog
{
namespace hwbc
{
using namespace ocs2;
using namespace ocs2::legged_robot;

struct HierarchicalWbcConfig
{
    double friction_coefficient = 0.3;
    double swing_kp = 350.0;
    double swing_kd = 37.0;
    double swing_task_weight = 100.0;
    double base_height_kp = 400.0;
    double base_height_kd = 140.0;
    double base_linear_kp = 400.0;
    double base_linear_kd = 100.0;
    double base_angular_kp = 400.0;
    double base_angular_kd = 140.0;
    // Direct posture regularization for the hip ab/adduction joints. The
    // NMPC state cost alone is weak once WBC must satisfy stance-foot rigidity
    // and base-height/angular tracking, so this adds an explicit qddot target
    // for HAA without making it a hard contact/dynamics constraint.
    double haa_posture_kp = 60.0;
    double haa_posture_kd = 8.0;
    double haa_posture_task_weight = 20.0;
    // Optional [LF, LH, RF, RH] HAA targets in actuated joint order. Empty
    // means track the MPC joint state as before.
    std::vector<double> haa_posture_nominal_rad;
    // Soft posture regularization for the full 12-joint leg shape. This is
    // useful for robots whose nominal hip/knee signs differ from the A1
    // source project: stance/swing/base tasks can otherwise find a dynamically
    // valid but mechanically ugly crouched posture.
    double leg_posture_kp = 0.0;
    double leg_posture_kd = 0.0;
    double leg_posture_task_weight = 0.0;
    // Optional 12-joint target in actuated joint order [LF, LH, RF, RH].
    // Empty means track the MPC joint state as before.
    std::vector<double> leg_posture_nominal_rad;
    // qm_control keeps the dog motor-facing WBC output disabled for the first
    // 10 s and uses a lighter WBC hierarchy during that window. StateTrot can
    // shorten this in simulation, but the default mirrors the reference.
    double init_task_seconds = 10.0;
    double torque_limit_scale = 1.0;
    // Optional per-leg [abad, hip, knee] WBC torque limits. Empty means use
    // Pinocchio/URDF effort limits multiplied by torque_limit_scale.
    std::vector<double> leg_torque_limits_nm;
    // Joint position-limit avoidance (see formulateJointLimitsTask): the QP
    // bounds each joint's optimized acceleration so that, extrapolated over
    // this horizon at current position/velocity, it would not cross the
    // URDF position limit. Not qm_control-derived - added because babyDog's
    // much shorter legs (vs. qm_control's Aliengo) let the ported swing/base
    // tasks command joint targets past the real URDF limits, which Gazebo's
    // physical joint stop then enforces as a hard, jarring collision instead
    // of the QP itself planning a smooth approach.
    double joint_limit_horizon_seconds = 0.15;
};

class WbcBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
            const PinocchioEndEffectorKinematics& eeKinematics, HierarchicalWbcConfig config = {});
    virtual ~WbcBase() = default;

    virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired,
                            const vector_t& rbdStateMeasured, size_t mode, scalar_t period, scalar_t time);

protected:
    void updateMeasured(const vector_t& rbdStateMeasured);
    void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
    vector_t updateCmd(vector_t x_optimal);

    size_t getNumDecisionVars() const { return numDecisionVars_; }

    Task formulateFloatingBaseEomTask();
    Task formulateTorqueLimitsTask();
    Task formulateNoContactMotionTask();
    Task formulateFrictionConeTask();
    Task formulateJointLimitsTask();

    Task formulateBaseHeightMotionTask();
    Task formulateBaseAngularMotionTask();
    Task formulateBaseLinearMotionTask();
    Task formulateHaaJointPostureTask();
    Task formulateLegJointPostureTask();
    Task formulateSwingLegTask();

    Task formulateContactForceTask(const vector_t& inputDesired) const;

    HierarchicalWbcConfig config_;

private:
    size_t numDecisionVars_;
    PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
    CentroidalModelInfo info_;
    CentroidalModelPinocchioMapping mapping_;
    std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

    contact_flag_t contactFlag_{};
    size_t numContacts_{};

    vector_t qMeasured_, vMeasured_, inputLast_;
    vector_t qDesired_, vDesired_, baseAccDesired_;
    vector_t jointAccel_;
    matrix_t j_, dj_;
    matrix_t base_j_, base_dj_;

    // Task parameters not carried in HierarchicalWbcConfig: derived from the
    // Pinocchio model itself (per-joint effort/position limits), not tunable
    // gains. Position limits are per-joint (unlike the uniform effort rating,
    // abad limits mirror sign per leg side - see CLAUDE.md's "home = 0"
    // invariant), so all 12 are read directly rather than one leg replicated.
    vector_t legTorqueLimits_;
    vector_t jointLowerLimits_, jointUpperLimits_;
};

}  // namespace hwbc
}  // namespace megadog

#endif  // CONTROLLER_CONTROL_WBCBASE_H
