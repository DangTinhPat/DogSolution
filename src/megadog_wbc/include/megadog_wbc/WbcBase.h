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

#include <array>
#include <limits>
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
    // When haa_posture_nominal_rad is set AND this is > 0, the task doesn't
    // pin the joint to the nominal angle exactly - it clamps the dynamic
    // qJointDesired target to [nominal - band, nominal + band] instead. This
    // gives the same kind of natural, swing-responsive hip motion as an
    // empty nominal (fully dynamic tracking), but keeps a bounded anchor so
    // the joint can never drift arbitrarily far from the configured anchor
    // (see formulateHaaJointPostureTask). 0.0 (default) preserves the old
    // binary behavior: fully pinned when nominal is set, fully dynamic when
    // it's empty.
    double haa_posture_dynamic_band_rad = 0.0;
    // Optional devq-style excursion guard. In the normal HAA range the posture
    // row follows the NMPC target at haa_posture_safe_scale. As either the
    // measured or planned angle approaches the configured guard magnitude,
    // the row smoothly returns to full authority and blends its target back
    // toward the configured nominal. Disabled by default to preserve the
    // upstream/static posture semantics for other users of this library.
    bool haa_posture_adaptive_guard_enabled = false;
    double haa_posture_safe_scale = 1.0;
    double haa_posture_guard_start_abs_rad = 0.44;
    double haa_posture_guard_full_abs_rad = 0.48;
    // Optional swing-foot lateral target shaping in contact order
    // [LF, RF, LH, RH]. This modifies formulateSwingLegTask()'s own Cartesian
    // target before the swing task is built, instead of adding a second foot-y
    // task that would compete with swing XYZ tracking. Values are body-frame
    // lateral foot offsets; positive is robot-left.
    double swing_foot_lateral_target_blend = 0.0;
    std::vector<double> swing_foot_lateral_nominal_y_m;
    // NOTE: deliberately no haa_posture_swing_scale field here (unlike
    // leg_posture_swing_scale below) - contact-gating this task the same
    // way is intentionally kept out of the current devq runtime: HAA posture
    // is used as a continuous stance-width anchor, while leg_posture has its
    // own contact-aware blending below.
    // Soft posture regularization for the full 12-joint leg shape. This is
    // useful for robots whose nominal hip/knee signs differ from the A1
    // source project: stance/swing/base tasks can otherwise find a dynamically
    // valid but mechanically ugly crouched posture.
    double leg_posture_kp = 0.0;
    double leg_posture_kd = 0.0;
    double leg_posture_task_weight = 0.0;
    // Per-leg scale applied to formulateLegJointPostureTask()'s pull for
    // whichever leg is currently SWINGING (contactFlag_[leg]==false) -
    // full weight (1.0) is always used for a leg currently in stance,
    // regardless of this field. A swinging leg's target ALSO switches to
    // the dynamic qJointDesired fallback regardless of any configured
    // nominal (see formulateLegJointPostureTask()), so this reduced weight
    // reinforces formulateSwingLegTask()'s own trajectory rather than
    // fighting it - a soft consistency check against upper-joint-stop
    // disturbance during swing, not a competitor.
    //
    // Six same-session sim attempts at a dynamic/bounded posture target
    // that still pulled on swinging legs at FULL weight (toward a moving
    // instead of fixed target) all destabilized trot within seconds to
    // tens of seconds - this field exists to remove that competition.
    // Zeroing it entirely (fully disabling posture during swing, no
    // backup at all) was tried first and verified clean through 332s of
    // TROT_IN_PLACE - but MIT WBIC's real-hardware precedent for "trust
    // the swing Cartesian task alone" (arXiv:1909.06586, Mini-Cheetah)
    // assumes a leg Jacobian that stays well-conditioned through the whole
    // swing range. Devq's KFE joint-frame rotation means q=0 is a physical
    // upper stop, not the collinear-leg singularity itself; it still leaves
    // only about 0.436 rad of geometric margin, less than A1-family models.
    // A swing trajectory disturbed toward that low-leverage stop therefore
    // still needs a restoring pull. Default 0.15 mirrors IIT DLS's published
    // phase-gated postural task (Raiola et al. 2020, Frontiers in
    // Robotics and AI: Kp_sw/Kd_sw meaningfully reduced from Kp_st/Kd_st,
    // not zeroed) - small enough to stay well clear of the fighting that
    // destabilized every full-weight attempt, but nonzero so a knee that
    // drifts toward the upper stop during swing still has *something*
    // pulling it back rather than nothing.
    double leg_posture_swing_scale = 0.15;
    // Optional 12-joint target in actuated joint order [LF, LH, RF, RH].
    // Empty means track the MPC joint state as before.
    std::vector<double> leg_posture_nominal_rad;
    // Optional 12-joint per-joint band, same semantics and same
    // [LF,LH,RF,RH]x[HAA,HFE,KFE] order as leg_posture_nominal_rad (see
    // haa_posture_dynamic_band_rad above for the shared mechanism this
    // mirrors). For a joint whose entry is > 0 (and leg_posture_nominal_rad
    // has a finite value at that index), the target is
    // clamp(qJointDesired, nominal-band, nominal+band) instead of pinned
    // exactly to nominal - natural, swing-responsive motion bounded around
    // the configured anchor. A joint's entry being 0/absent keeps that one
    // joint's old exact-pin behavior. This is deliberately per-joint rather
    // than a single scalar: devq's KFE has less geometric margin near its
    // upper stop than the A1-family models and needs a tight band, while HFE
    // has generous physical margin on both sides and can safely take a
    // wider one.
    std::vector<double> leg_posture_dynamic_band_rad;
    // Optional phase-continuous posture regularization for devq. HAA rows are
    // omitted here because formulateHaaJointPostureTask() owns them. HFE/KFE
    // blend the NMPC and nominal targets in stance, while KFE smoothly
    // regains the full nominal anchor as it approaches its upper joint stop.
    // Contact changes are blended to avoid a one-tick 0.15 -> 1.0 weight jump.
    bool leg_posture_adaptive_guard_enabled = false;
    double leg_posture_stance_scale = 1.0;
    // Fraction of the NMPC joint target retained after a leg has fully
    // entered stance. 1.0 follows NMPC exactly; 0.0 restores the fixed
    // nominal anchor. The same contact blend used for row authority also
    // blends this target, avoiding a discontinuity at touchdown/liftoff.
    double leg_posture_stance_nmpc_blend = 1.0;
    double leg_posture_contact_blend_seconds = 0.0;
    double leg_posture_kfe_guard_start_rad = -0.70;
    double leg_posture_kfe_guard_full_rad = -0.45;
    // qm_control keeps the dog motor-facing WBC output disabled for the first
    // 10 s and uses a lighter WBC hierarchy during that window. StateTrot can
    // shorten this in simulation, but the default mirrors the reference.
    double init_task_seconds = 10.0;
    double torque_limit_scale = 1.0;
    // Optional per-leg [abad, hip, knee] WBC torque limits. Empty means use
    // Pinocchio/URDF effort limits multiplied by torque_limit_scale.
    std::vector<double> leg_torque_limits_nm;
    // Joint position-limit avoidance (see formulateJointLimitsTask): a
    // relative-degree-2 control barrier function (CBF) on h(q)=limit-q,
    // enforcing ḧ >= -k2*ḣ - k1*h (i.e. qddot bounded so the QP can never
    // choose an acceleration that lets the joint reach the limit). Not
    // qm_control-derived - added because babyDog's much shorter legs (vs.
    // qm_control's Aliengo) let the ported swing/base tasks command joint
    // targets past the real URDF limits, which Gazebo's physical joint stop
    // then enforces as a hard, jarring collision instead of the QP itself
    // planning a smooth approach.
    double joint_limit_horizon_seconds = 0.15;
    // k1 in the CBF above is derived from joint_limit_horizon_seconds as
    // k1=2/horizon^2 (unchanged from the original "reach the limit by time
    // horizon under constant velocity" intuition). k2 = 2*damping_ratio*
    // sqrt(k1). The ORIGINAL implementation implicitly used damping_ratio=
    // 1/sqrt(2)=0.707 (k2=2/horizon) - a genuine bug found this session:
    // that gives a complex-conjugate pole pair for the h(t) dynamics
    // (Hurwitz/stable, but NOT "totally negative" - see Kurtz/Wensing/Lin,
    // arXiv:2109.13349, Theorem 2, on the extra condition an ECBF's gain
    // matrix needs beyond ordinary stability), so the worst-case h(t) is a
    // decaying OSCILLATION that can swing past h=0 (i.e. past the actual
    // position limit) before damping out, rather than approaching it
    // monotonically - confirmed in sim as a knee joint sustaining a
    // violation of its configured hard bound for 28+ continuous seconds
    // during trot (oscillating around/past the limit) despite the
    // constraint being "hard" in the QP. damping_ratio>=1.0 (real,
    // non-positive poles - critically damped at exactly 1.0, overdamped
    // above) is required for the barrier to actually be forward-invariant;
    // this is clamped to >=1.0 in formulateJointLimitsTask() regardless of
    // what's configured here, as a hard floor against reintroducing the
    // bug. Default 1.2 is mildly overdamped for a small safety margin
    // beyond the critical-damping minimum.
    double joint_limit_damping_ratio = 1.2;
    // Optional per-joint TIGHTENING of the bound formulateJointLimitsTask()
    // enforces, in the same [LF,LH,RF,RH]x[HAA,HFE,KFE] order as
    // leg_posture_nominal_rad. A finite entry here can only pull that
    // joint's effective bound INWARD from the real URDF/Pinocchio limit
    // (never loosen past it - see the WbcBase.cpp constructor, which clamps
    // each override into [lowerPositionLimit, upperPositionLimit]) - this
    // is a hard inequality constraint at a QP priority level ABOVE the
    // swing/posture/base tasks (taskJointLimits, HierarchicalWbc.cpp), so
    // unlike a posture task's soft target it cannot be out-voted by a
    // higher-effective-priority task at the same level.
    //
    // Exists specifically because real reference robots (A1/Go1/Aliengo -
    // see qiayuanl/legged_control's own URDFs) all keep 37-52.5 degrees of
    // built-in URDF margin between their own calf_position_max and the
    // thigh/calf-collinear singularity. Devq's KFE frame offset leaves about
    // 0.436 rad to collinearity at q=0, but that is still a smaller margin.
    // This override lets megaDog restore
    // the same kind of real, hardware-enforced-style margin real quadruped
    // URDFs carry, at the QP-constraint level, without touching the actual
    // URDF/hardware limit itself (which may or may not reflect a genuine
    // mechanical stop - unconfirmed for devq).
    std::vector<double> joint_position_upper_limits_override_rad;
    std::vector<double> joint_position_lower_limits_override_rad;
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
    void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period, size_t mode);
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
    std::array<scalar_t, 4> legPostureStanceBlend_{};
    bool legPostureBlendInitialized_ = false;

    vector_t qMeasured_, vMeasured_, inputLast_;
    vector_t qDesired_, vDesired_, baseAccDesired_;
    // Raw one-tick finite difference of inputDesired (see updateDesired()),
    // used as feedforward for baseAccDesired_. inputDesired's per-leg
    // contact-force components can jump near-discontinuously across a
    // contact-mode switch (diagonal swap), which this difference divides by
    // a ~1ms(sim)/5ms(real) period - amplifying that jump by up to ~1000x
    // into a one-tick spike, felt as a phantom "jerk the body" pulse right
    // at each phase transition (root cause of a user-reported dip during
    // trot, found by root-cause investigation - see updateDesired()'s
    // comment for the fix). modeLast_ lets updateDesired() detect exactly
    // that one tick and hold jointAccel_ at zero instead of differentiating
    // across the discontinuity - std::numeric_limits<size_t>::max() as the
    // initial sentinel deliberately can't match any real ModeNumber, so the
    // very first tick after construction is also treated as a "mode
    // change" (safe: inputLast_ is zero then too, so the real difference
    // would have been ~0 anyway).
    vector_t jointAccel_;
    size_t modeLast_ = std::numeric_limits<size_t>::max();
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
