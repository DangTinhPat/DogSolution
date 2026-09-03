#include "megadog_controller/MegadogController.h"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace megadog_controller
{
namespace
{
using config_type = controller_interface::interface_configuration_type;

// Matches megadog_legged_interface's actuatedDofNum order exactly (see
// megadog_description/urdf/robot.xacro's leg instantiation order LF, LH, RF, RH).
// devq vs. A1 scaling factors (see const.xacro's real mass/length numbers):
//   length_ratio = devq's thigh/calf_length (0.15) / A1's (0.2) = 0.75
//   leg_inertia_ratio = per-leg mass ratio (1.112684/1.694 = 0.657) *
//                        length_ratio^2 (0.5625) = 0.3696 - approximates how
//                        much less reflected inertia a devq leg joint sees
//                        vs. A1's, for gains that bypass WBC's mass matrix.
// kJointKd runs on top of WbcBase's QP output as a direct joint-space torque
// term (torque += kJointKd * velocity_error) - it does NOT go through
// Pinocchio's mass matrix like WbcBase's own Cartesian kp/kd do, so it scales
// with leg_inertia_ratio (was legged_control's stock kd=3 for A1-scale legs).
constexpr double kJointKd = 2.0;
constexpr double kTorqueLimitNm = 80.0;  // sim debug limit; intentionally above devq's physical rating.
constexpr double kHaaTorqueLimitNm = 80.0;
// NOTE: was documented as "comfortably under reference.info's
// targetDisplacementVelocity (0.5 m/s)" - corrected this session
// (nmpc-expert investigation): megaDog's actual reference.info value is 0.3,
// not 0.5 (0.5 was apparently a different repo's number), AND that field is
// dead configuration for this runtime path anyway - grep-confirmed only
// LeggedRobotPoseCommandNode.cpp/TargetTrajectoriesPublisher.cpp (a
// different, unused entry point) read it; MegadogWbcRuntime::
// setTargetTrajectories() computes its own timeToTarget from
// mpc.timeHorizon directly (task.info) and never touches
// targetDisplacementVelocity/targetRotationVelocity. kWalkSpeedMps is not
// actually bounded by anything in reference.info - it's simply the
// deliberately-chosen forward/backward walking speed.
constexpr double kWalkSpeedMps = 0.12;
// Max |d(velocity)/dt| applied to the FSM's target velocity before it reaches
// MegadogWbcCommand::base_velocity_x_m_s - see update()'s ramp toward
// target_velocity_x. Without this, switching TROT_IN_PLACE -> FORWARD (or the
// reverse) stepped the commanded velocity from 0 to kWalkSpeedMps in a single
// control tick; setTargetTrajectories() folds that straight into a stepped
// target *position* timeToTarget seconds out, which the MPC then has to
// "catch up to" abruptly - visible as a jerky stutter rather than a
// smooth speed-up/slow-down. At this rate, a full forward<->backward reversal
// (2*kWalkSpeedMps) takes 2*kWalkSpeedMps/kVelocityRampMps2 = 1.2s.
//
// This same "step-command -> jerky stutter" mechanism is why kStrafeSpeedMps/
// kTurnRateRadS below get their OWN dedicated ramp constants rather than
// reusing this one - researcher-agent-confirmed: task.info's sqp.sqpIteration=1
// (Real-Time-Iteration-style NMPC, one Newton step per solve, not iterated to
// convergence) does NOT "absorb" a stepped reference within a single control
// tick the way a fully-converged solver might get closer to doing (Diehl,
// Bock & Schlöder 2005's RTI theory - a reference discontinuity is absorbed
// gradually over several solves, not immediately) - so every commanded axis
// genuinely needs its own explicit ramp, this is not just a cosmetic nicety.
constexpr double kVelocityRampMps2 = 0.4;
// Lateral (crab-walk) and yaw-rate (turning) locomotion - added after
// forward/backward, deliberately conservative starting values since this
// exercises a fundamentally different (frontal-plane, for lateral) balance
// mode never verified in this codebase before. Multi-agent research
// (researcher/locomotion-command-expert/
// nmpc-expert/trot-gait-expert, this session) cross-checked against
// qiayuanl/legged_control, unitree_guide, and MIT Cheetah 3's convex-MPC
// paper (Di Carlo et al., IROS 2018) all confirm: (vx, vy, wz) is the
// canonical 3-DOF command interface for this class of controller, gait.info's
// mode template needs zero changes for lateral/rotational motion (foot
// placement is a free NMPC optimization result, not a fixed heuristic tied to
// a direction), and every reference sets a narrower lateral envelope than
// forward (MIT Cheetah 3: 3 m/s fwd vs 1 m/s lateral vs 180 deg/s yaw;
// unitree_guide's A1/Go1 config: ~75% of forward) - the concrete numbers
// don't transfer (different platform/geometry), but the qualitative pattern
// (raise these only after a dedicated lateral/turning HAA/KFE excursion
// stress test, and keep them well under kWalkSpeedMps/a full-speed turn)
// does. Simultaneous multi-axis commands (e.g. forward+turn at once) are
// deliberately OUT of scope for this first implementation - each new FSM
// state below drives exactly one axis, so the "omnidirectional motion
// anisotropy" combined-magnitude infeasibility prior art warns about
// (Zhang, Xu, Cai & Zhu, arXiv:2403.10101) cannot arise here by construction,
// not by an added rejection check - deferred to a future arc-motion feature.
constexpr double kStrafeSpeedMps = 0.10;
constexpr double kLateralVelocityRampMps2 = 0.4;
// Was 0.3 rad/s (~17.2 deg/s); earlier sim runs showed sustained turning
// demanded larger HAA excursions than forward or lateral motion. Keep this
// conservative until turn-specific HAA/collision-distance logs are refreshed.
constexpr double kTurnRateRadS = 0.12;
constexpr double kYawRateRampRadS2 = 0.3;
constexpr double kComHeightM = 0.22;
constexpr double kStandupDurationS = 3.0;
constexpr double kStandupKp = 35.0;
constexpr double kStandupKd = 1.8;
constexpr double kWbcHfePostureKp = 0.0;
constexpr double kWbcKfePostureKp = 0.0;
constexpr double kDiagnosticsPeriodS = 2.0;
// Duration effort is ramped (via smoothStep()) from last_valid_effort_
// toward the freshly-computed WBC torque once runtime_->update() resumes
// succeeding after a hold - e.g. the ~20ms/kMpcResetSettleAdvances-tick
// hold MegadogWbcRuntime imposes while waiting for a fresh MPC policy
// after beginNewLocomotionSegment() (STAND->TROT_IN_PLACE and other
// locomotion-segment entries). Without this, effort snapped in a single
// control tick from STAND's frozen stance torque to trot's first active
// torque (root-caused this session as the trigger of a "wobbles briefly
// before settling into trot" symptom - see effort_hold_active_'s doc
// comment in MegadogController.h). 50ms is a few multiples of the nominal
// ~20ms hold (so it fully covers the actual gap, not just part of it) but
// small relative to base_height/linear/angular_kp/kd's own ~0.3-0.7s
// overdamped settling time - it only removes the initiating kick, it
// doesn't meaningfully add to how long the (separately unavoidable, not
// touched here) settling itself takes.
constexpr double kEffortBlendDurationS = 0.05;

// HAA nominal is a tunable stance-width parameter. Keep task.info's
// initialState and reference.info's defaultJointState in sync with this.
// FK on the current devq URDF shows that increasing HAA magnitude narrows
// the foot track because of devq's HAA origin roll and lateral offsets.
// This pass moves 0.40 -> 0.43 rad as a supervised sim experiment; do not
// carry this to hardware until collision-distance and sustained-gait logs
// have been refreshed.
//
// HFE/KFE bumped 0.574027/-1.37275 -> 0.478618/-1.181561: the user noticed
// STAND settling visibly higher than TROT_IN_PLACE despite both commanding
// the same comHeight=0.22 (kComHeightM below). urdf-expert's FK analysis
// found the OLD HFE/KFE values were never actually kinematically consistent
// with comHeight=0.22 - they naturally produce a foot-frame height of
// 0.23876m (URDF frame, ~19mm too high). STAND hard-pins all 4 legs to
// this posture at once (formulateLegJointPostureTask, weight 22), so it
// wins over the single base-height task and settles near the posture's
// own natural height; TROT_IN_PLACE only pins 2 legs (the current stance
// diagonal) at a time, so it's pulled closer to the true 0.22 target -
// hence the visible STAND/TROT height mismatch. The new values were
// solved via FK (Newton iteration, holding foot x/y fixed) to land
// exactly at 0.22000m for all 4 legs, so STAND and TROT should now settle
// at consistent heights. Lateral foot spread barely changes (+0.75mm).
// KFE's static clearance to the calf_position_max=0.0 upper stop shrinks
// from 1.373 to 1.182 rad (~14%) - still far from the ~0.80-0.87 rad
// band-edges documented as unsafe in this session's earlier failed
// dynamic-band experiments (see leg_posture_dynamic_band_rad's history
// below), but re-verified via a dedicated stress test (see git log/commit
// message for this change) specifically watching KFE_meas peak excursion,
// since dynamic swing has been shown to outrun the static nominal before.
//
// A second, SEPARATE, ~0.02m systematic offset was also found (foot
// collision sphere radius=0.0205m not accounted for in the NMPC's
// point-foot kinematic model, vs. ground-truth measurement using the true
// physical - sphere-inclusive - base height) - this affects STAND and
// TROT equally, so it doesn't cause the mismatch fixed above, and is out
// of scope here (would need a fix in the measurement/terrain-height path,
// not joint angles) - flagged, not fixed.
constexpr std::array<double, 12> kStandingJointTargetRad{
    -0.43, 0.478618, -1.181561,
    -0.43, 0.478618, -1.181561,
     0.43, 0.478618, -1.181561,
     0.43, 0.478618, -1.181561,
};

bool isHaaJointIndex(const std::size_t joint_index)
{
    return joint_index % 3 == 0;
}

double smoothStep(const double x)
{
    const double t = std::clamp(x, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double smoothStepDerivative(const double x)
{
    const double t = std::clamp(x, 0.0, 1.0);
    return 6.0 * t * (1.0 - t);
}

double clampJointTorque(const std::size_t joint_index, const double torque)
{
    const double limit = isHaaJointIndex(joint_index) ? kHaaTorqueLimitNm : kTorqueLimitNm;
    return std::isfinite(torque) ? std::clamp(torque, -limit, limit) : 0.0;
}

// WbcBase's Cartesian task gains (swing_kp/kd, base_*_kp/kd in WbcBase.h)
// compute a *desired acceleration* (b = accDesired + kp*posError + kd*velError)
// that WbcBase's QP then maps to torque through Pinocchio's real mass matrix
// - so unlike kJointKd above, these do NOT need mass scaling (the QP
// already uses devq's real, ported mass/inertia).
//
// (A same-session experiment tried making haa_posture/leg_posture track a
// *dynamic* target (empty nominal_rad, falls back to qJointDesired each
// tick in formulateHaaJointPostureTask()/formulateLegJointPostureTask(),
// WbcBase.cpp) instead of the fixed standing-pose angles below, aiming for
// a less "robotic" hip swing like ultraDog's own HierarchicalWbcConfig{}
// defaults. Doing this for BOTH tasks at once, plus raising swing_kp/kd/
// base_angular_kp/kd to ultraDog's values, made TROT_IN_PLACE fall over
// within ~8-10s in sim (leg_posture's fixed nominal turned out to be the
// only thing anchoring the (HFE,KFE) null-space away from devq's
// calf_position_max=0.0 upper stop - losing it removed that protection).
// A narrower follow-up tried the dynamic target on HAA alone (no
// kinematic-singularity risk there), with leg_posture/swing/base gains
// left untouched, and knocked haa_posture_kp/kd/weight down from
// 120/20/80 to 90/15/30 - this passed 104s of isolated headless-sim
// TROT_IN_PLACE with no instability by every logged metric (eom residual,
// roll/pitch/yaw, torque, joint limits). It still looked worse in the
// user's own live testing than the original fixed-nominal behavior below -
// a real discrepancy between what this session's sim metrics can see and
// what actually looks natural, not something to dismiss. Fully reverted;
// do not re-attempt a dynamic haa_posture/leg_posture target without a
// genuinely different mechanism, and don't trust headless sim metrics
// alone as sufficient evidence for this specific "looks natural" axis -
// get live/visual confirmation before considering it validated.
megadog::hwbc::HierarchicalWbcConfig makeDevqWbcConfig()
{
    megadog::hwbc::HierarchicalWbcConfig config;
    // Was WbcBase.h's default (0.3, A1-stock, never explicitly set here)
    // until it was found to mismatch megadog_world.sdf's actual simulated
    // ground friction (mu=1.0) by >3x - see kStandingJointTargetRad's
    // comment above (attempt 5) for why this was investigated and what it
    // did/didn't fix. 0.5 stays comfortably under the sim's real mu=1.0.
    // Must be kept in lock-step with task.info's
    // frictionConeSoftConstraint.frictionCoefficient (also 0.5) - the WBC's
    // own friction cone task and the NMPC's belief must agree, or the NMPC
    // could commit to a trajectory relying on more lateral GRF than this
    // task1-priority QP constraint actually allows the WBC to deliver.
    config.friction_coefficient = 0.5;
    config.swing_kp = 260.0;
    config.swing_kd = 28.0;
    config.swing_task_weight = 55.0;
    config.base_height_kp = 400.0;
    config.base_height_kd = 140.0;
    config.base_linear_kp = 400.0;
    config.base_linear_kd = 100.0;
    // Tried raising to stock 400.0/140.0 this session as step 2 of fixing
    // lateral-strafe yaw drift (after task.info's theta_base_z Q-weight
    // 100->300, step 1). Verified via a controlled STAND->STRAFE_LEFT sim
    // A/B (matched initial conditions, delta measured relative to the
    // pre-strafe STAND heading, not raw yaw - STAND's own anchor drifts
    // with whatever heading it's entered from, so absolute yaw isn't
    // comparable across runs): induced yaw delta stayed ~0.04-0.05 rad
    // across all three configurations tested (baseline, +Q(9,9) alone,
    // +Q(9,9)+this gain) - raising this gain provided no measurable
    // benefit. Reverted to avoid the unnecessary risk of touching a gain
    // family with documented (if unrelated-cause) fall history in this
    // codebase, for zero measured gain. TROT_IN_PLACE regression at 400/140
    // was itself clean (29s+, no instability) - this revert is about
    // "no benefit, don't carry the risk," not a re-discovered instability.
    // The residual ~0.04-0.05 rad heading drift during sustained lateral
    // strafe is a real, characterized, SAFE (roll/pitch/eom/HAA/KFE all
    // stayed nominal throughout every test) limitation of this first
    // implementation - most likely a genuine physical yaw moment from
    // asymmetric diagonal-pair GRF lever arms during Y-direction
    // acceleration that neither lever tested here fully cancels. Not
    // resolved this session; a real fix would need a different mechanism
    // (e.g. NMPC-level structural change, or investigating whether it's
    // specific to devq's stance asymmetry) - flagged as follow-up work,
    // not attempted further here per the conservative-first-pass scope.
    config.base_angular_kp = 300.0;
    config.base_angular_kd = 105.0;
    config.haa_posture_kp = 120.0;
    config.haa_posture_kd = 20.0;
    config.haa_posture_task_weight = 80.0;
    // HAA nominal narrows devq's stance as its magnitude increases. The
    // current 0.43 rad setting is intentionally staged for supervised sim
    // validation before any hardware use. kp/kd/weight stay at the values
    // that were stable with the previous 0.40 rad nominal.
    //
    // haa_posture_dynamic_band_rad below lets this task track the WBC's own
    // moving qJointDesired within +-band of the nominal, instead of pinning
    // it exactly. Keep the band tight enough that forward trot cannot let
    // HAA relax back toward a wider footprint for long stretches.
    config.haa_posture_dynamic_band_rad = 0.006;
    config.haa_posture_nominal_rad = {-0.43, -0.43, 0.43, 0.43};
    config.haa_posture_adaptive_guard_enabled = false;
    config.haa_posture_safe_scale = 1.0;
    config.haa_posture_guard_start_abs_rad = 0.47;
    config.haa_posture_guard_full_abs_rad = 0.50;
    // Experimental hook for reshaping only the SWING foot's own Cartesian
    // target; kept disabled until a supervised A/B shows a clear width win.
    config.swing_foot_lateral_target_blend = 0.0;
    config.swing_foot_lateral_nominal_y_m = {0.112, -0.112, 0.112, -0.112};
    // leg_posture_task_weight must stay > 0 (see the long comment above) -
    // this is the one non-negotiable devq-specific WBC gain found this
    // session, everything else here is otherwise-standard devq tuning
    // predating today's ground-truth/GaitSchedule-mutex fixes.
    config.leg_posture_kp = 65.0;
    config.leg_posture_kd = 10.0;
    config.leg_posture_task_weight = 22.0;
    config.leg_posture_nominal_rad = {
        -0.43, 0.478618, -1.181561,
        -0.43, 0.478618, -1.181561,
         0.43, 0.478618, -1.181561,
         0.43, 0.478618, -1.181561,
    };
    config.leg_posture_adaptive_guard_enabled = true;
    config.leg_posture_stance_scale = 1.0;
    config.leg_posture_stance_nmpc_blend = 0.25;
    config.leg_posture_contact_blend_seconds = 0.05;
    config.leg_posture_kfe_guard_start_rad = -0.70;
    config.leg_posture_kfe_guard_full_rad = -0.45;
    // A same-session experiment tried bounded-dynamic bands on HFE/KFE too
    // (HFE 0.40, KFE 0.50 rad around nominal, FK-derived to leave 0.873 rad
    // clear of the calf_position_max=0.0 upper stop) - STAND was fully
    // stable for 114s, but TROT_IN_PLACE was NOT: KFE breached the intended
    // band within 14s (measured -0.817, past the -0.87275 edge) and by
    // ~184s in, KFE hit exactly 0.0 - the physical upper stop - causing a
    // full tumble (roll spiked to 1.31 rad, torque saturated). Root cause
    // (confirmed by research this session comparing against upstream
    // qiayuanl/legged_control and real A1/Go1/Aliengo URDFs): the band only
    // bounds the *target* fed into leg_posture's soft, weighted QP task
    // (weight 22, same priority level as swing/base) - it does not
    // hard-clamp the actual solved joint position, so swing/stance dragged
    // the real KFE trajectory well past the clamped target.
    //
    // A SECOND attempt re-enabled this band, now backed by a genuine hard
    // safety net the first attempt didn't have:
    // joint_position_upper_limits_override_rad (below) tightens
    // taskJointLimits - a QP priority level ABOVE task1's swing/posture
    // competition, so it can't be out-voted the way leg_posture's own soft
    // clamp was - to KFE<=-0.80. This also failed: clean through t~200s
    // (past the first attempt's ~184s failure point), then a warning
    // appeared in the log ("inconsistent mode schedule: eventTimes=16
    // modeSequence=17, falling back to STANCE") immediately followed by a
    // torque spike (15-29 Nm vs. the normal ~3-5 Nm) and a full tumble -
    // and during that tumble, KFE_meas AND KFE_mpc (the QP's own commanded
    // target) both measured above -0.80, i.e. the hard constraint itself
    // was violated in practice, then KFE hit exactly 0.0 again. Lesson: a
    // hard QP inequality constrains the *commanded* acceleration each
    // solve tick, not a guarantee on the *actual physical* joint position
    // once the robot is already destabilizing (torque saturating, high
    // angular rates) - it's necessary but evidently not sufficient once
    // something else has already knocked the robot off balance. The
    // "inconsistent mode schedule" warning was traced to its actual root
    // cause and fixed: MegadogWbcRuntime::update() was reading the current
    // gait mode through ocs2::ReferenceManager's own unguarded
    // BufferedValue<ModeSchedule> (a SEPARATE object from GaitSchedule's
    // already-mutex-protected one) from the control thread, while the MPC
    // thread concurrently mutated that same buffer via preSolverRun() - the
    // exact same cross-thread pattern the GaitSchedule mutex above was
    // built to prevent, just on a different, previously-missed object (see
    // GaitSchedule.h's mutex_ doc comment for the full story). Fixed via
    // GaitSchedule::getCurrentModeSchedule() + switching both of
    // MegadogWbcRuntime.cpp's per-tick mode queries to read through it.
    // Verified via isolated sim: the warning did not occur once across
    // 359s of continuous TROT_IN_PLACE (vs. reliably by ~184-226s before),
    // and the robot stayed fully stable throughout - strong evidence this,
    // not the joint-band mechanism itself, was the actual trigger behind
    // both prior tumbles.
    //
    // A THIRD attempt re-enabled this band with the race fixed AND the hard
    // KFE<=-0.80 constraint in place - and STILL failed: clean (and no
    // "inconsistent mode schedule" warning at all this time, confirming the
    // race fix holds) until ~t=232s into TROT_IN_PLACE, when RF leg's
    // KFE_meas AND KFE_mpc both sustained well above -0.80 (oscillating
    // -0.71..-0.78) for 28+ continuous seconds before the run was stopped
    // (per the monitoring protocol, before it could cascade into a full
    // tumble like the earlier failing run). So the race was never the only
    // problem:
    // the "hard" QP inequality in formulateJointLimitsTask() itself does
    // not actually bound the real trajectory here. Most likely explanation
    // not yet confirmed: it's a horizon-extrapolated (not exact) bound -
    // qddotMax = (2/horizon^2)*(jointUpperLimits_ - qJoint - vJoint*horizon)
    // assumes ~constant velocity over joint_limit_horizon_seconds=0.15s.
    // Root cause now actually found and fixed (not just hypothesized):
    // formulateJointLimitsTask()'s constraint is algebraically a
    // relative-degree-2 control barrier function (CBF) on h=limit-q, and
    // the original derivation's implicit gain pair gave h(t) a pair of
    // COMPLEX-conjugate poles (Hurwitz/stable, but not "totally negative" -
    // research this session cites Kurtz/Wensing/Lin arXiv:2109.13349's
    // ECBF gain condition) - i.e. the barrier's own worst-case trajectory
    // is a decaying OSCILLATION that can swing past the limit before
    // damping out, exactly matching the observed "oscillating -0.71..-0.78
    // for 28+ seconds" symptom. Fixed via
    // HierarchicalWbcConfig::joint_limit_damping_ratio (WbcBase.h/.cpp),
    // clamped to >=1.0 (real poles) - see that field's doc comment for the
    // full derivation.
    //
    // A FOURTH attempt re-enabled this band with the CBF gain fix above -
    // and failed WORSE and FASTER than all three prior attempts: a full
    // base-orientation tumble (roll/pitch/yaw all diverging, base height
    // collapsing) within ~34s of TROT_IN_PLACE, vs. the ~184-232s the
    // earlier attempts survived. Critically, this failure's signature is
    // DIFFERENT from attempts 2/3's: KFE_mpc (the MPC's own commanded
    // target, not just the WBC's tracking of it) diverged along with
    // KFE_meas, and the WHOLE base orientation diverged within under 2s -
    // this looks like a genuine MPC/whole-body balance failure, not a
    // localized joint-limit-barrier oscillation. Suspected cause (not yet
    // confirmed): formulateJointLimitsTask() runs for ALL 12 joints, ALL
    // the time, not just KFE when an override is set - the damping_ratio
    // fix increased k2 (the velocity-damping gain) by roughly 70% network-
    // wide (from the old buggy implicit ratio 1/sqrt(2)=0.707 to the new
    // floor of 1.0), which could be throttling qddot more aggressively
    // than before during ordinary fast trot swings even far from any
    // actual position limit (the -k2*v term matters whenever velocity is
    // high, not just near a limit) - possibly fighting the swing task
    // hard enough to destabilize balance. This is a real, separate
    // regression risk introduced by the CBF fix itself when combined with
    // dynamic swing motion, not proof the fix's math is wrong. The
    // damping_ratio fix was since independently isolated (leg_posture
    // pinned, no KFE override) and verified safe alone across ~296s of
    // TROT_IN_PLACE - so it's not itself the destabilizer.
    //
    // A FIFTH attempt narrowed the experiment to HFE ALONE - KFE's band
    // entry left at 0 (exactly pinned, matching the always-proven-safe
    // fixed target), no KFE override set, so the joint every prior failure
    // involved was completely untouched. This STILL failed, and faster and
    // harder than every prior attempt: full tumble within ~10-14s of
    // TROT_IN_PLACE (vs. attempt 4's ~34s, the previous fastest), with
    // KFE_meas staying tight near its pinned nominal right up until the
    // tumble was already underway - i.e. KFE only diverged as a
    // CONSEQUENCE of the base losing balance, not a cause. This is
    // decisive: across 5 independent attempts (HFE+KFE together x4,
    // HFE-alone x1), EVERY dynamically-bounded leg_posture band has
    // destabilized trot within seconds to tens of seconds, including one
    // (HFE) with zero kinematic-singularity risk. The common factor across
    // all 5 is not KFE's singularity - it's leg_posture itself being a
    // SOFT, weighted QP task (weight 22) at the SAME priority tier as
    // swing/base (task1, HierarchicalWbc.cpp), competing rather than
    // being subordinate to them; a moving target at that tier apparently
    // lets swing/stance and the posture pull fight each other into
    // instability once real trot dynamics (not just STAND) are involved,
    // regardless of which joint or how carefully its target/gains are
    // chosen.
    //
    // A SIXTH attempt tried an architectural fix instead of another
    // tuning pass: moving posture to its own QP priority level strictly
    // below task1/task2 (null-space-projected, HierarchicalWbc.cpp) so it
    // could never compete with swing/base. This made things WORSE than
    // all 5 prior attempts - the robot couldn't even complete a stable
    // STAND (tumbled ~11.5s after the stand command, before trot was ever
    // reached). Root cause: during STAND, formulateSwingLegTask()
    // contributes nothing, so task1 alone leaves a large, largely
    // UNBOUNDED null-space direction in the 12 leg joints - the exact
    // drift direction leg_posture_task_weight was originally added to
    // fix (see the long comment above), which needs CONTINUOUS, WEIGHTED
    // authority to counter, not merely "whatever's left over after task1
    // is satisfied" (task1's own optimal solution doesn't care about that
    // direction at all, so the leftover slack there turned out to be far
    // LESS restraining than the old weighted competition, not more).
    // Reverted (HierarchicalWbc.cpp back to the original weighted-sum
    // task1). The real fix for TROT_IN_PLACE's failure mode likely needs
    // per-leg/per-contact-mode posture scaling (full weight for a stance
    // leg, near-zero for a leg that's actively swinging, where
    // formulateSwingLegTask() already fully determines its trajectory and
    // posture only adds friction) rather than a single global QP-tier
    // choice - a real, substantial follow-up, not attempted here.
    // The legacy dynamic-band path stays disabled. The adaptive path above
    // now provides a phase-continuous partial nominal anchor instead.
    config.leg_posture_dynamic_band_rad.clear();
    config.leg_torque_limits_nm = {80.0, 80.0, 80.0};
    // No fixed KFE override; the adaptive guard raises nominal authority
    // before the physical upper stop instead.
    // HAA uses its own posture task and NMPC-level soft box; no WBC hard
    // joint-position override is active for it here.
    config.joint_position_upper_limits_override_rad.clear();
    config.joint_position_lower_limits_override_rad.clear();
    // qm_control's full 10s warm-up leaves STAND/TROT running without base
    // height/angular/posture/swing tasks (HierarchicalWbc.cpp's task1) for
    // several seconds after handoff, so the optimizer can settle into
    // visually impossible-looking leg poses before the real posture tasks
    // ever engage - so 0.0 was used for a long time instead. But 0.0 makes
    // `if (time < config_.init_task_seconds)` permanently false (time>=0.0
    // always), which is a DEAD branch, not "no warm-up on this one
    // handoff" - it removes task1's ramp-in at EVERY locomotion-segment
    // entry (runtime_time_s resets to ~0 there), including STAND->TROT.
    // Root-caused this session as a second, independent contributor
    // (alongside MegadogController's own effort_hold_active_ blend above)
    // to a "wobbles briefly before settling into trot" symptom: task1
    // (base_height/linear/angular + haa/leg posture + swing, all at full
    // configured gain) engages at full weight the exact same tick its own
    // targets discontinuously jump from STAND's static reference to
    // trot's first optimized point. 0.15s is long enough to span the
    // ~20ms/kMpcResetSettleAdvances hold with margin, but far short of
    // the original 10s that caused the visually-bad-pose problem in the
    // first place - short enough that task0's redundant-DOF freedom
    // during the ramp shouldn't be visible as an ugly pose, only as a
    // gentler transition into task1 authority.
    config.init_task_seconds = 0.15;
    return config;
}

double quatToYaw(double w, double x, double y, double z)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
double quatToPitch(double w, double x, double y, double z)
{
    const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    return std::asin(sinp);
}
double quatToRoll(double w, double x, double y, double z)
{
    return std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
}

std::array<double, 3> bodyAngularVelocityToEulerZyxRate(const std::array<double, 3>& euler_zyx_rad,
                                                        const std::array<double, 3>& omega_body_rad_s)
{
    const double roll = euler_zyx_rad[2];
    const double pitch = euler_zyx_rad[1];
    const double wx = omega_body_rad_s[0];
    const double wy = omega_body_rad_s[1];
    const double wz = omega_body_rad_s[2];
    const double cos_pitch = std::cos(pitch);
    if (std::abs(cos_pitch) < 0.05 || !std::isfinite(cos_pitch)) {
        return {0.0, 0.0, 0.0};
    }
    const double sin_roll = std::sin(roll);
    const double cos_roll = std::cos(roll);
    const double tan_pitch = std::tan(pitch);
    const double roll_dot = wx + sin_roll * tan_pitch * wy + cos_roll * tan_pitch * wz;
    const double pitch_dot = cos_roll * wy - sin_roll * wz;
    const double yaw_dot = (sin_roll * wy + cos_roll * wz) / cos_pitch;
    return {yaw_dot, pitch_dot, roll_dot};
}

bool finiteArray(const std::array<double, 3>& values)
{
    return std::all_of(values.begin(), values.end(), [](const double value) { return std::isfinite(value); });
}

const char* fsmStateName(const MegadogFsmState state)
{
    switch (state) {
        case MegadogFsmState::HOME:
            return "HOME";
        case MegadogFsmState::STAND:
            return "STAND";
        case MegadogFsmState::STAND_NMPC:
            return "STAND_NMPC";
        case MegadogFsmState::STAND_WBC:
            return "STAND_WBC";
        case MegadogFsmState::TROT_IN_PLACE:
            return "TROT_IN_PLACE";
        case MegadogFsmState::FORWARD:
            return "FORWARD";
        case MegadogFsmState::BACKWARD:
            return "BACKWARD";
        case MegadogFsmState::STRAFE_LEFT:
            return "STRAFE_LEFT";
        case MegadogFsmState::STRAFE_RIGHT:
            return "STRAFE_RIGHT";
        case MegadogFsmState::TURN_LEFT:
            return "TURN_LEFT";
        case MegadogFsmState::TURN_RIGHT:
            return "TURN_RIGHT";
    }
    return "UNKNOWN";
}

bool isWbcState(const MegadogFsmState state)
{
    return state == MegadogFsmState::STAND || state == MegadogFsmState::STAND_NMPC ||
           state == MegadogFsmState::STAND_WBC || state == MegadogFsmState::TROT_IN_PLACE || state == MegadogFsmState::FORWARD ||
           state == MegadogFsmState::BACKWARD || state == MegadogFsmState::STRAFE_LEFT ||
           state == MegadogFsmState::STRAFE_RIGHT || state == MegadogFsmState::TURN_LEFT ||
           state == MegadogFsmState::TURN_RIGHT;
}

bool isLocomotionState(const MegadogFsmState state)
{
    return state == MegadogFsmState::TROT_IN_PLACE || state == MegadogFsmState::FORWARD ||
           state == MegadogFsmState::BACKWARD || state == MegadogFsmState::STRAFE_LEFT ||
           state == MegadogFsmState::STRAFE_RIGHT || state == MegadogFsmState::TURN_LEFT ||
           state == MegadogFsmState::TURN_RIGHT;
}
}  // namespace

const std::vector<std::string>& MegadogController::jointNames()
{
    static const std::vector<std::string> names = {
        "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
    };
    return names;
}

controller_interface::InterfaceConfiguration MegadogController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(jointNames().size());
    for (const auto& joint_name : jointNames()) {
        conf.names.push_back(joint_name + "/effort");
    }
    return conf;
}

controller_interface::InterfaceConfiguration MegadogController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(jointNames().size() * 2);
    for (const auto& joint_name : jointNames()) {
        conf.names.push_back(joint_name + "/position");
        conf.names.push_back(joint_name + "/velocity");
    }
    return conf;
}

controller_interface::CallbackReturn MegadogController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MegadogController::on_configure(const rclcpp_lifecycle::State&)
{
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

    // A MutuallyExclusive callback group serializes invocations in arrival
    // order, so consecutive callback calls never race on
    // latest_sim_base_state_ writes.
    sim_base_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sim_base_options;
    sim_base_options.callback_group = sim_base_callback_group_;
    sim_base_subscription_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
        "/sim/model_poses", rclcpp::SensorDataQoS(),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg)
        {
            // The ros_gz_bridge conversion for this topic does not populate
            // child_frame_id, so the base link can't be matched by name -
            // index 0 is used instead (SDF/Gazebo emits poses in model/link
            // declaration order, and "base" is A1's first declared link in
            // robot.xacro). If base_fresh diagnostics ever look wrong (e.g.
            // z height tracking a leg instead of the trunk), re-derive the
            // correct index empirically.
            if (msg->transforms.empty()) {
                return;
            }
            const geometry_msgs::msg::TransformStamped* base = &msg->transforms[0];
            const double qw = base->transform.rotation.w;
            const double qx = base->transform.rotation.x;
            const double qy = base->transform.rotation.y;
            const double qz = base->transform.rotation.z;
            const double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            const bool finite_pose = std::isfinite(base->transform.translation.x) &&
                std::isfinite(base->transform.translation.y) && std::isfinite(base->transform.translation.z) &&
                std::isfinite(q_norm) && q_norm > 0.5;
            if (!finite_pose) {
                return;
            }
            const double inv_norm = 1.0 / q_norm;
            const double nw = qw * inv_norm, nx = qx * inv_norm, ny = qy * inv_norm, nz = qz * inv_norm;

            SimBaseSample next;
            next.position_m = {base->transform.translation.x, base->transform.translation.y,
                               base->transform.translation.z};
            next.euler_zyx_rad = {quatToYaw(nw, nx, ny, nz), quatToPitch(nw, nx, ny, nz), quatToRoll(nw, nx, ny, nz)};
            next.orientation_wxyz = {nw, nx, ny, nz};
            next.stamp = std::chrono::steady_clock::now();
            next.has_data = true;

            std::lock_guard<std::mutex> lock(sim_base_mutex_);
            // Freshness only needs a recent pose sample - velocity is a
            // best-effort finite difference against whatever the previous
            // sample was, skipped only when dt is too small to divide by
            // safely. Messages can arrive in tight bursts, so dt between two
            // consecutive callback invocations is not a reliable proxy for
            // the topic's true update period; requiring a "reasonable" dt
            // window here just made freshness flicker without adding safety.
            if (latest_sim_base_state_.has_data) {
                const double dt = std::chrono::duration<double>(next.stamp - latest_sim_base_state_.stamp).count();
                if (dt > 1e-6) {
                    for (int i = 0; i < 3; ++i) {
                        next.linear_velocity_m_s[i] = (next.position_m[i] - latest_sim_base_state_.position_m[i]) / dt;
                        double delta = next.euler_zyx_rad[i] - latest_sim_base_state_.euler_zyx_rad[i];
                        delta = std::atan2(std::sin(delta), std::cos(delta));
                        next.euler_zyx_rate_rad_s[i] = delta / dt;
                    }
                }
            }
            latest_sim_base_state_ = next;
        },
        sim_base_options);

    real_imu_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions real_imu_options;
    real_imu_options.callback_group = real_imu_callback_group_;
    real_imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            const double qw = msg->orientation.w;
            const double qx = msg->orientation.x;
            const double qy = msg->orientation.y;
            const double qz = msg->orientation.z;
            const double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            const std::array<double, 3> omega_body{
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
            };
            const std::array<double, 3> linear_accel{
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
            };
            const bool finite_imu = std::isfinite(q_norm) && q_norm > 0.5 &&
                finiteArray(omega_body) && finiteArray(linear_accel);
            if (!finite_imu) {
                return;
            }
            const double inv_norm = 1.0 / q_norm;
            const double nw = qw * inv_norm, nx = qx * inv_norm, ny = qy * inv_norm, nz = qz * inv_norm;

            RealImuSample next;
            next.euler_zyx_rad = {quatToYaw(nw, nx, ny, nz), quatToPitch(nw, nx, ny, nz), quatToRoll(nw, nx, ny, nz)};
            next.orientation_wxyz = {nw, nx, ny, nz};
            next.angular_velocity_body_rad_s = omega_body;
            next.linear_acceleration_m_s2 = linear_accel;
            next.euler_zyx_rate_rad_s = bodyAngularVelocityToEulerZyxRate(next.euler_zyx_rad, omega_body);
            next.stamp = std::chrono::steady_clock::now();
            next.has_data = finiteArray(next.euler_zyx_rad) && finiteArray(next.euler_zyx_rate_rad_s);
            if (!next.has_data) {
                return;
            }

            std::lock_guard<std::mutex> lock(imu_mutex_);
            latest_real_imu_state_ = next;
        },
        real_imu_options);

    cmd_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cmd_options;
    cmd_options.callback_group = cmd_callback_group_;
    cmd_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
        "/megadog/cmd", rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(get_node()->get_logger(), "MegadogController: received /megadog/cmd '%s'", msg->data.c_str());
            MegadogFsmState next;
            if (msg->data == "home") {
                next = MegadogFsmState::HOME;
            } else if (msg->data == "stand") {
                next = MegadogFsmState::STAND;
            } else if (msg->data == "stand_nmpc") {
                next = MegadogFsmState::STAND_NMPC;
            } else if (msg->data == "stand_wbc") {
                next = MegadogFsmState::STAND_WBC;
            } else if (msg->data == "trot_in_place") {
                next = MegadogFsmState::TROT_IN_PLACE;
            } else if (msg->data == "forward" || msg->data == "trot" || msg->data == "trot_forward") {
                next = MegadogFsmState::FORWARD;
            } else if (msg->data == "backward") {
                next = MegadogFsmState::BACKWARD;
            } else if (msg->data == "strafe_left") {
                next = MegadogFsmState::STRAFE_LEFT;
            } else if (msg->data == "strafe_right") {
                next = MegadogFsmState::STRAFE_RIGHT;
            } else if (msg->data == "turn_left") {
                next = MegadogFsmState::TURN_LEFT;
            } else if (msg->data == "turn_right") {
                next = MegadogFsmState::TURN_RIGHT;
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Unknown /megadog/cmd '%s' (expected home|stand|stand_nmpc|stand_wbc|trot_in_place|forward|trot|trot_forward|backward|strafe_left|strafe_right|turn_left|turn_right)",
                             msg->data.c_str());
                return;
            }
            // HOME's settled prone pose is the requested init pose, so normal
            // commands hand directly to WBC/MPC.
            fsm_state_.store(static_cast<int>(next), std::memory_order_relaxed);
        },
        cmd_options);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MegadogController::on_activate(const rclcpp_lifecycle::State&)
{
    // OCS2/WBC runtime is created lazily on the first non-HOME command. This
    // keeps opening the simulator in the passive HOME pose from touching CppAD/
    // MPC at all, which avoids startup aborts before the user asks the robot to
    // stand or walk.
    runtime_.reset();
    start_time_s_ = -1.0;
    elapsed_s_ = 0.0;
    wbc_time_s_ = 0.0;
    diagnostics_elapsed_s_ = 0.0;
    runtime_failure_reported_ = false;
    time_jump_reported_ = false;
    // Always start at rest - see MegadogFsmState::HOME's doc comment. A
    // previous activation's state (if any) is deliberately not preserved.
    fsm_state_.store(static_cast<int>(MegadogFsmState::HOME), std::memory_order_relaxed);
    last_fsm_state_seen_ = MegadogFsmState::HOME;
    last_valid_effort_.fill(0.0);
    has_last_valid_effort_ = false;
    effort_hold_active_ = false;
    effort_blend_elapsed_s_ = 0.0;
    latched_base_position_reference_m_ = {};
    latched_base_yaw_reference_rad_ = 0.0;
    base_reference_latched_ = false;
    last_control_base_sample_ = {};
    last_control_base_sample_valid_ = false;
    filtered_base_linear_velocity_m_s_ = {};
    filtered_base_euler_zyx_rate_rad_s_ = {};
    state_entered_wbc_time_s_ = 0.0;
    locomotion_runtime_epoch_wbc_time_s_ = 0.0;
    smoothed_velocity_x_m_s_ = 0.0;
    smoothed_velocity_y_m_s_ = 0.0;
    smoothed_yaw_rate_rad_s_ = 0.0;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MegadogController::on_deactivate(const rclcpp_lifecycle::State&)
{
    runtime_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MegadogController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    const double dt = period.seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
        for (auto& command_interface : command_interfaces_) {
            std::ignore = command_interface.set_value(0.0);
        }
        return controller_interface::return_type::OK;
    }
    constexpr double kMaxControllerPeriodS = 0.05;
    if (dt > kMaxControllerPeriodS) {
        if (!time_jump_reported_) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "MegadogController: abnormal control period %.6fs; holding effort and resetting WBC/MPC time state",
                        dt);
            time_jump_reported_ = true;
        }
        if (runtime_) {
            runtime_->beginNewLocomotionSegment();
        }
        last_control_base_sample_valid_ = false;
        filtered_base_linear_velocity_m_s_ = {};
        filtered_base_euler_zyx_rate_rad_s_ = {};
        const auto held_effort = has_last_valid_effort_ ? last_valid_effort_ : std::array<double, 12>{};
        for (std::size_t j = 0; j < jointNames().size(); ++j) {
            std::ignore = command_interfaces_[j].set_value(held_effort[j]);
        }
        return controller_interface::return_type::OK;
    }
    time_jump_reported_ = false;
    elapsed_s_ += dt;
    if (start_time_s_ < 0.0) {
        start_time_s_ = 0.0;
    }

    SimBaseSample base_sample;
    RealImuSample imu_sample;
    bool base_fresh;
    bool imu_fresh;
    {
        std::lock_guard<std::mutex> lock(sim_base_mutex_);
        base_sample = latest_sim_base_state_;
        const auto now = std::chrono::steady_clock::now();
        base_fresh = base_sample.has_data && std::chrono::duration<double>(now - base_sample.stamp).count() < 0.2;
    }
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_sample = latest_real_imu_state_;
        const auto now = std::chrono::steady_clock::now();
        imu_fresh = imu_sample.has_data && std::chrono::duration<double>(now - imu_sample.stamp).count() < 0.2;
    }

    megadog::hwbc::MegadogWbcMeasurement measurement;
    std::array<double, 12> measured_velocity{};
    for (std::size_t j = 0; j < jointNames().size(); ++j) {
        const auto position = state_interfaces_[2 * j].get_optional();
        const auto velocity = state_interfaces_[2 * j + 1].get_optional();
        const double pos = position && std::isfinite(*position) ? *position : 0.0;
        const double vel = velocity && std::isfinite(*velocity) ? *velocity : 0.0;
        measurement.joint_pos_rad[j] = pos;
        measurement.joint_vel_rad_s[j] = vel;
        measured_velocity[j] = vel;
    }

    if (base_fresh) {
        // Ground truth available (sim only) - use it directly, exactly like
        // ultraDog does, with no estimator in the loop at all. See
        // base_pose_is_ground_truth's doc comment in MegadogWbcRuntime.h for
        // why MegadogWbcRuntime must be told this explicitly rather than
        // inferring it from which fields are populated.
        if (last_control_base_sample_valid_ && base_sample.stamp > last_control_base_sample_.stamp) {
            const double base_dt =
                std::chrono::duration<double>(base_sample.stamp - last_control_base_sample_.stamp).count();
            if (base_dt > 1e-4 && base_dt < 0.2) {
                constexpr double kBaseVelocityFilterAlpha = 0.35;
                for (int i = 0; i < 3; ++i) {
                    const double raw_linear =
                        (base_sample.position_m[i] - last_control_base_sample_.position_m[i]) / base_dt;
                    double raw_euler = base_sample.euler_zyx_rad[i] - last_control_base_sample_.euler_zyx_rad[i];
                    raw_euler = std::atan2(std::sin(raw_euler), std::cos(raw_euler)) / base_dt;
                    if (std::isfinite(raw_linear)) {
                        filtered_base_linear_velocity_m_s_[i] +=
                            kBaseVelocityFilterAlpha * (raw_linear - filtered_base_linear_velocity_m_s_[i]);
                    }
                    if (std::isfinite(raw_euler)) {
                        filtered_base_euler_zyx_rate_rad_s_[i] +=
                            kBaseVelocityFilterAlpha * (raw_euler - filtered_base_euler_zyx_rate_rad_s_[i]);
                    }
                }
            }
        } else if (!last_control_base_sample_valid_) {
            filtered_base_linear_velocity_m_s_ = {};
            filtered_base_euler_zyx_rate_rad_s_ = {};
        } else {
            for (int i = 0; i < 3; ++i) {
                filtered_base_linear_velocity_m_s_[i] *= 0.995;
                filtered_base_euler_zyx_rate_rad_s_[i] *= 0.995;
            }
        }
        if (!last_control_base_sample_valid_ || base_sample.stamp > last_control_base_sample_.stamp) {
            last_control_base_sample_ = base_sample;
            last_control_base_sample_valid_ = true;
        }

        measurement.base_pos_m = base_sample.position_m;
        measurement.base_euler_zyx_rad = imu_fresh ? imu_sample.euler_zyx_rad : base_sample.euler_zyx_rad;
        measurement.base_linear_vel_m_s = filtered_base_linear_velocity_m_s_;
        measurement.base_euler_zyx_rate_rad_s =
            imu_fresh ? imu_sample.euler_zyx_rate_rad_s : filtered_base_euler_zyx_rate_rad_s_;
        measurement.base_linear_accel_local_m_s2 = imu_fresh ? imu_sample.linear_acceleration_m_s2 : std::array<double, 3>{};
        measurement.base_pose_is_ground_truth = true;

        // Same ground-truth pose, republished as a standard "odom" -> "base"
        // TF - see odom_tf_broadcaster_'s doc comment in the header for why
        // this exists (RViz's Fixed Frame can be "odom" instead of always
        // "base", so the body's actual walking motion through the world is
        // visible instead of only the legs moving relative to a pinned base).
        if (odom_tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = time;
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base";
            odom_tf.transform.translation.x = base_sample.position_m[0];
            odom_tf.transform.translation.y = base_sample.position_m[1];
            odom_tf.transform.translation.z = base_sample.position_m[2];
            const auto& orientation = imu_fresh ? imu_sample.orientation_wxyz : base_sample.orientation_wxyz;
            odom_tf.transform.rotation.w = orientation[0];
            odom_tf.transform.rotation.x = orientation[1];
            odom_tf.transform.rotation.y = orientation[2];
            odom_tf.transform.rotation.z = orientation[3];
            odom_tf_broadcaster_->sendTransform(odom_tf);
        }
    } else if (imu_fresh) {
        // No ground truth (real hardware, or sim without /sim/model_poses
        // bridged): attitude/angular rate/linear acceleration come from
        // /imu/data. base_pos_m/base_linear_vel_m_s here are just
        // placeholders - MegadogWbcRuntime's BaseStateEstimator overwrites
        // both every tick with its own leg-odometry estimate (IMU + joint
        // encoders + gait-schedule contact), since base_pose_is_ground_truth
        // is left false below.
        measurement.base_pos_m = {0.0, 0.0, kComHeightM};
        measurement.base_euler_zyx_rad = imu_sample.euler_zyx_rad;
        measurement.base_linear_vel_m_s = {0.0, 0.0, 0.0};
        measurement.base_euler_zyx_rate_rad_s = imu_sample.euler_zyx_rate_rad_s;
        measurement.base_linear_accel_local_m_s2 = imu_sample.linear_acceleration_m_s2;
        last_control_base_sample_valid_ = false;
        filtered_base_linear_velocity_m_s_ = {};
        filtered_base_euler_zyx_rate_rad_s_ = {};

        if (odom_tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = time;
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base";
            odom_tf.transform.translation.x = measurement.base_pos_m[0];
            odom_tf.transform.translation.y = measurement.base_pos_m[1];
            odom_tf.transform.translation.z = measurement.base_pos_m[2];
            odom_tf.transform.rotation.w = imu_sample.orientation_wxyz[0];
            odom_tf.transform.rotation.x = imu_sample.orientation_wxyz[1];
            odom_tf.transform.rotation.y = imu_sample.orientation_wxyz[2];
            odom_tf.transform.rotation.z = imu_sample.orientation_wxyz[3];
            odom_tf_broadcaster_->sendTransform(odom_tf);
        }
    } else {
        measurement.base_pos_m = {0.0, 0.0, kComHeightM};
        last_control_base_sample_valid_ = false;
        filtered_base_linear_velocity_m_s_ = {};
        filtered_base_euler_zyx_rate_rad_s_ = {};
    }

    const auto fsm_state = static_cast<MegadogFsmState>(fsm_state_.load(std::memory_order_relaxed));
    const auto previous_fsm_state = last_fsm_state_seen_;
    if (previous_fsm_state == MegadogFsmState::HOME && fsm_state != MegadogFsmState::HOME) {
        wbc_time_s_ = 0.0;
    } else if (fsm_state != MegadogFsmState::HOME) {
        wbc_time_s_ += dt;
    }
    if (fsm_state != previous_fsm_state) {
        state_entered_wbc_time_s_ = wbc_time_s_;
        if (previous_fsm_state == MegadogFsmState::HOME && fsm_state != MegadogFsmState::HOME) {
            standup_start_joint_pos_ = measurement.joint_pos_rad;
            standup_start_latched_ = true;
        }
        if (isLocomotionState(fsm_state) && !isLocomotionState(previous_fsm_state)) {
            locomotion_runtime_epoch_wbc_time_s_ =
                std::max(state_entered_wbc_time_s_, standup_start_latched_ ? kStandupDurationS : 0.0);
            // The upcoming locomotion segment's local clock (runtime_time_s
            // below) restarts at/near 0 here, but neither the MPC solver's
            // own internal warm-start state nor the WBC runtime's gait-
            // schedule validity bookkeeping reset on their own - found via
            // stress-testing this session (re-entering a locomotion state
            // after a STAND hold, which suspends the MPC worker, froze the
            // gait into a near-static pose, reproduced 3/3 times) and fixed
            // only after two attempts, once the actual root cause (a stale
            // gait-schedule validity window, not just the MPC solver) was
            // found. See MegadogWbcRuntime::beginNewLocomotionSegment()'s
            // doc comment for the full story.
            if (runtime_) {
                runtime_->beginNewLocomotionSegment();
            }
        }
    }

    if (fsm_state == MegadogFsmState::HOME) {
        base_reference_latched_ = false;
        standup_start_latched_ = false;
        locomotion_runtime_epoch_wbc_time_s_ = 0.0;
    } else if (isWbcState(fsm_state) && (!base_reference_latched_ || fsm_state != previous_fsm_state)) {
        latched_base_position_reference_m_ = measurement.base_pos_m;
        latched_base_yaw_reference_rad_ = measurement.base_euler_zyx_rad[0];
        base_reference_latched_ = true;
    }

    // Ramp toward the FSM state's target velocity every tick (including
    // HOME/STAND, so it decays back to 0 smoothly instead of snapping - see
    // kVelocityRampMps2's doc comment for why this matters). Runs
    // unconditionally for all three axes regardless of which axis (if any)
    // the current fsm_state actually drives, so leaving e.g. STRAFE_LEFT for
    // TURN_RIGHT ramps y back toward 0 while yaw-rate ramps up - no axis is
    // ever left stuck at a stale nonzero value across a mode switch.
    const double target_velocity_x = fsm_state == MegadogFsmState::FORWARD    ? kWalkSpeedMps
                                      : fsm_state == MegadogFsmState::BACKWARD ? -kWalkSpeedMps
                                                                                : 0.0;
    const double target_velocity_y = fsm_state == MegadogFsmState::STRAFE_LEFT    ? kStrafeSpeedMps
                                      : fsm_state == MegadogFsmState::STRAFE_RIGHT ? -kStrafeSpeedMps
                                                                                    : 0.0;
    const double target_yaw_rate = fsm_state == MegadogFsmState::TURN_LEFT    ? kTurnRateRadS
                                    : fsm_state == MegadogFsmState::TURN_RIGHT ? -kTurnRateRadS
                                                                                 : 0.0;
    const double max_delta = kVelocityRampMps2 * dt;
    smoothed_velocity_x_m_s_ += std::clamp(target_velocity_x - smoothed_velocity_x_m_s_, -max_delta, max_delta);
    const double max_delta_y = kLateralVelocityRampMps2 * dt;
    smoothed_velocity_y_m_s_ += std::clamp(target_velocity_y - smoothed_velocity_y_m_s_, -max_delta_y, max_delta_y);
    const double max_delta_yaw = kYawRateRampRadS2 * dt;
    smoothed_yaw_rate_rad_s_ += std::clamp(target_yaw_rate - smoothed_yaw_rate_rad_s_, -max_delta_yaw, max_delta_yaw);

    std::array<double, 12> effort{};
    bool ok = false;
    megadog::hwbc::MegadogWbcResult result;
    if (fsm_state == MegadogFsmState::HOME) {
        // HOME is the prone/resting init pose: literal zero effort, no WBC/MPC
        // call. The next locomotion command first uses the short joint-space
        // stand-up ramp below, then hands control to WBC/MPC.
        runtime_failure_reported_ = false;
    } else if (standup_start_latched_ && wbc_time_s_ < kStandupDurationS) {
        const double phase = wbc_time_s_ / kStandupDurationS;
        const double alpha = smoothStep(phase);
        const double alpha_dot = smoothStepDerivative(phase) / kStandupDurationS;
        for (std::size_t j = 0; j < jointNames().size(); ++j) {
            const double delta = kStandingJointTargetRad[j] - standup_start_joint_pos_[j];
            const double pos_des = standup_start_joint_pos_[j] + alpha * delta;
            const double vel_des = alpha_dot * delta;
            const double torque =
                kStandupKp * (pos_des - measurement.joint_pos_rad[j]) + kStandupKd * (vel_des - measured_velocity[j]);
            effort[j] = clampJointTorque(j, torque);
            result.position_rad[j] = pos_des;
            result.velocity_rad_s[j] = vel_des;
            result.torque_nm[j] = effort[j];
        }
        result.valid = true;
        ok = true;
        runtime_failure_reported_ = false;
    } else {
        megadog::hwbc::MegadogWbcCommand command;
        command.com_height_m = kComHeightM;
        switch (fsm_state) {
            case MegadogFsmState::STAND:
            case MegadogFsmState::STAND_NMPC:
            case MegadogFsmState::STAND_WBC:
                command.gait_name = "stance";
                command.use_mpc_for_stance_hold = fsm_state == MegadogFsmState::STAND_NMPC;
                if (fsm_state != MegadogFsmState::STAND_NMPC && base_reference_latched_) {
                    command.base_x_reference_m = latched_base_position_reference_m_[0];
                    command.base_y_reference_m = latched_base_position_reference_m_[1];
                    command.base_yaw_reference_rad = latched_base_yaw_reference_rad_;
                }
                break;
            case MegadogFsmState::TROT_IN_PLACE:
            case MegadogFsmState::FORWARD:
            case MegadogFsmState::BACKWARD:
            case MegadogFsmState::STRAFE_LEFT:
            case MegadogFsmState::STRAFE_RIGHT:
            case MegadogFsmState::TURN_LEFT:
            case MegadogFsmState::TURN_RIGHT:
                // Keep all trot-family states on the same phase template.
                // Forward/backward stability is handled by reducing the
                // commanded speed/ramp below the in-place gait's kinematic
                // comfort limit, not by changing phase timing mid-family.
                command.gait_name = "trot";
                command.base_velocity_x_m_s = smoothed_velocity_x_m_s_;
                command.base_velocity_y_m_s = smoothed_velocity_y_m_s_;
                command.base_yaw_rate_rad_s = smoothed_yaw_rate_rad_s_;
                if (base_reference_latched_) {
                    // Only hard-anchor an axis THIS state doesn't intend to
                    // move on (gated on the discrete per-state target, not
                    // the ramping smoothed value, to avoid float-equality
                    // flicker mid-ramp). x is never anchored here (was
                    // already true before strafe/turn existed) - FORWARD/
                    // BACKWARD need it free-running, and setTargetTrajectories()
                    // already free-runs x from the current base pose each
                    // tick when base_x_reference_m is left non-finite. The
                    // same free-run/anchor split now applies per-axis: y
                    // stays anchored (no lateral drift) unless STRAFE_* is
                    // actively driving it, and yaw stays anchored (no
                    // unintended turning/heading drift) unless TURN_* is
                    // actively driving it - this is what makes STRAFE_LEFT
                    // genuine sideways translation with heading held fixed,
                    // not a turn-then-walk.
                    if (target_velocity_y == 0.0) {
                        command.base_y_reference_m = latched_base_position_reference_m_[1];
                    }
                    if (target_yaw_rate == 0.0) {
                        command.base_yaw_reference_rad = latched_base_yaw_reference_rad_;
                    }
                }
                break;
            case MegadogFsmState::HOME:
                break;  // unreachable (handled above)
        }

        const double runtime_epoch_s =
            (fsm_state == MegadogFsmState::STAND || fsm_state == MegadogFsmState::STAND_NMPC ||
             fsm_state == MegadogFsmState::STAND_WBC) &&
                    standup_start_latched_
                ? kStandupDurationS
                : locomotion_runtime_epoch_wbc_time_s_;
        const double runtime_time_s = std::max(0.0, wbc_time_s_ - runtime_epoch_s);
        if (!base_fresh && !imu_fresh) {
            if (!runtime_failure_reported_) {
                RCLCPP_WARN(
                    get_node()->get_logger(),
                    "MegadogController has no fresh /sim/model_poses or /imu/data; WBC/NMPC held");
                runtime_failure_reported_ = true;
            }
            ok = false;
        } else if (!runtime_) {
            runtime_ = std::make_unique<megadog::hwbc::MegadogWbcRuntime>(makeDevqWbcConfig(), true);
            if (!runtime_->ready()) {
                RCLCPP_ERROR(get_node()->get_logger(), "MegadogWbcRuntime failed to initialize");
                runtime_.reset();
            }
        }
        if (base_fresh || imu_fresh) {
            ok = runtime_ && runtime_->ready() && runtime_->update(runtime_time_s, dt, time, measurement, command, result);
        }
        if (ok && result.valid) {
            for (std::size_t j = 0; j < jointNames().size(); ++j) {
                double posture_torque = 0.0;
                if (j % 3 == 1) {
                    posture_torque = kWbcHfePostureKp * (kStandingJointTargetRad[j] - measurement.joint_pos_rad[j]);
                } else if (j % 3 == 2) {
                    posture_torque = kWbcKfePostureKp * (kStandingJointTargetRad[j] - measurement.joint_pos_rad[j]);
                }
                const double torque = result.torque_nm[j] + kJointKd * (result.velocity_rad_s[j] - measured_velocity[j]) +
                                      posture_torque;
                effort[j] = clampJointTorque(j, torque);
            }
            if (effort_hold_active_) {
                // Ramp from the frozen hold value toward this tick's fresh
                // torque instead of snapping to it in one tick - see
                // kEffortBlendDurationS's doc comment above.
                effort_blend_elapsed_s_ += dt;
                const double alpha = smoothStep(effort_blend_elapsed_s_ / kEffortBlendDurationS);
                for (std::size_t j = 0; j < jointNames().size(); ++j) {
                    effort[j] = last_valid_effort_[j] + alpha * (effort[j] - last_valid_effort_[j]);
                }
                if (alpha >= 1.0) {
                    effort_hold_active_ = false;
                }
            }
            last_valid_effort_ = effort;
            has_last_valid_effort_ = true;
            runtime_failure_reported_ = false;
        } else {
            if (!runtime_failure_reported_) {
                if (base_fresh || imu_fresh) {
                    RCLCPP_WARN(get_node()->get_logger(), "MegadogWbcRuntime produced no valid sample; holding last valid effort");
                }
                runtime_failure_reported_ = true;
            }
            effort = has_last_valid_effort_ ? last_valid_effort_ : effort;
            // Arms the blend above for whenever a fresh sample resumes -
            // re-primed every tick of the hold so the blend always starts
            // cleanly from elapsed=0 the instant the hold actually ends.
            effort_hold_active_ = has_last_valid_effort_;
            effort_blend_elapsed_s_ = 0.0;
        }
    }

    for (std::size_t j = 0; j < jointNames().size(); ++j) {
        std::ignore = command_interfaces_[j].set_value(effort[j]);
    }

    diagnostics_elapsed_s_ += dt;
    if (diagnostics_elapsed_s_ >= kDiagnosticsPeriodS) {
        diagnostics_elapsed_s_ = 0.0;
        RCLCPP_INFO(
            get_node()->get_logger(),
            "MegadogController: t=%.2f wbc_t=%.2f state=%s valid=%d base_src=%s imu_fresh=%d "
            "cmd=[vx %.3f vy %.3f wz %.3f] eom=%.4f "
            "base=[z %.3f yaw %.3f pitch %.3f roll %.3f vx %.3f vy %.3f vz %.3f wyaw %.3f wpitch %.3f wroll %.3f] "
            "feet_y_body=[LF %.3f RF %.3f LH %.3f RH %.3f] "
            "track_width=[front %.3f hind %.3f mean %.3f skew %.3f diag %.3f left_skew %.3f right_skew %.3f] "
            "HAA_meas=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HFE_meas=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "KFE_meas=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HAA_mpc=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HFE_mpc=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "KFE_mpc=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HAA_tau=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "HFE_tau=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "KFE_tau=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "HAA_eff=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "HFE_eff=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "KFE_eff=[LF %.2f LH %.2f RF %.2f RH %.2f]",
            elapsed_s_, wbc_time_s_, fsmStateName(fsm_state), ok && result.valid ? 1 : 0,
            base_fresh ? "gt" : (imu_fresh ? "imu" : "none"), imu_fresh ? 1 : 0,
            smoothed_velocity_x_m_s_, smoothed_velocity_y_m_s_, smoothed_yaw_rate_rad_s_, result.eom_residual_norm,
            measurement.base_pos_m[2], measurement.base_euler_zyx_rad[0], measurement.base_euler_zyx_rad[1],
            measurement.base_euler_zyx_rad[2], measurement.base_linear_vel_m_s[0], measurement.base_linear_vel_m_s[1],
            measurement.base_linear_vel_m_s[2],
            measurement.base_euler_zyx_rate_rad_s[0], measurement.base_euler_zyx_rate_rad_s[1],
            measurement.base_euler_zyx_rate_rad_s[2],
            result.foot_lateral_y_m[0], result.foot_lateral_y_m[1], result.foot_lateral_y_m[2],
            result.foot_lateral_y_m[3],
            result.front_width_m, result.hind_width_m, result.width_mean_m, result.width_skew_m,
            result.active_diagonal_width_m, result.left_fore_hind_skew_m, result.right_fore_hind_skew_m,
            measurement.joint_pos_rad[0], measurement.joint_pos_rad[3], measurement.joint_pos_rad[6],
            measurement.joint_pos_rad[9],
            measurement.joint_pos_rad[1], measurement.joint_pos_rad[4], measurement.joint_pos_rad[7],
            measurement.joint_pos_rad[10],
            measurement.joint_pos_rad[2], measurement.joint_pos_rad[5], measurement.joint_pos_rad[8],
            measurement.joint_pos_rad[11],
            result.position_rad[0], result.position_rad[3], result.position_rad[6], result.position_rad[9],
            result.position_rad[1], result.position_rad[4], result.position_rad[7], result.position_rad[10],
            result.position_rad[2], result.position_rad[5], result.position_rad[8], result.position_rad[11],
            result.torque_nm[0], result.torque_nm[3], result.torque_nm[6], result.torque_nm[9],
            result.torque_nm[1], result.torque_nm[4], result.torque_nm[7], result.torque_nm[10],
            result.torque_nm[2], result.torque_nm[5], result.torque_nm[8], result.torque_nm[11],
            effort[0], effort[3], effort[6], effort[9], effort[1], effort[4], effort[7], effort[10], effort[2],
            effort[5], effort[8], effort[11]);
    }

    last_fsm_state_seen_ = fsm_state;

    return controller_interface::return_type::OK;
}

}  // namespace megadog_controller

PLUGINLIB_EXPORT_CLASS(megadog_controller::MegadogController, controller_interface::ControllerInterface)
