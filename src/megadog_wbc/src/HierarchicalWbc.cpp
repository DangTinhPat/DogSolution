// Ported from skywoodsz/qm_control's qm_wbc::HierarchicalWbc.cpp - see
// include/megadog_wbc/HierarchicalWbc.h for what was stripped.

#include "megadog_wbc/HierarchicalWbc.h"

#include "megadog_wbc/HoQp.h"

#include <cmath>

namespace megadog
{
namespace hwbc
{

HierarchicalWbc::HierarchicalWbc(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                 const PinocchioEndEffectorKinematics& eeKinematics, HierarchicalWbcConfig config)
    : WbcBase(pinocchioInterface, std::move(info), eeKinematics, config)
{
}

vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                                 size_t mode, scalar_t period, scalar_t time)
{
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);

    // task0 is true physics (EOM, torque limits, rigid stance, friction cone)
    // - these must be combined into a single least-squares/slack QP because
    // they are all fundamentally simultaneous requirements, so none of them
    // can be allowed to "lose" to another. Joint-limit avoidance is not
    // physics, just a safety preference, so it is kept as its OWN, separate,
    // lower-priority HoQp level (taskJointLimits) solved in task0's null
    // space - this way EOM/contact rigidity are never sacrificed for it, but
    // it still outranks base/swing tracking (task1) and force/xy tracking
    // (task2) when those would otherwise drive a joint past its URDF limit.
    Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateNoContactMotionTask() +
                 formulateFrictionConeTask();
    Task taskJointLimits = formulateJointLimitsTask();
    Task task2 = formulateContactForceTask(inputDesired) + formulateBaseLinearMotionTask();

    if (time < config_.init_task_seconds) {
        HoQp hoQp(task2, std::make_shared<HoQp>(taskJointLimits, std::make_shared<HoQp>(task0)));
        vector_t x_optimal = hoQp.getSolutions();
        return WbcBase::updateCmd(x_optimal);
    }

    // A same-session experiment tried moving posture (leg_posture +
    // haa_posture) OUT of task1 into its own, strictly-lower-priority
    // null-space-projected level (below both task1 and task2), on the
    // theory that summing it into task1 as an equally-weighted competitor
    // of swing/base was what let it fight swing into instability during
    // trot (5 prior joint-flexibility attempts all failed that way).
    // Verified via isolated sim: this made things WORSE, not better - the
    // robot could no longer even complete a stable STAND (tumbled within
    // ~11.5s of the stand command, before trot was ever attempted).
    // Root cause understood in hindsight: during STAND, formulateSwingLegTask()
    // contributes nothing (no leg is swinging), so task1 is JUST base
    // height/angular - a low-dimensional constraint that leaves a large,
    // largely UNBOUNDED null-space direction in the 12 leg joints (this is
    // the exact same drift direction an original, much earlier experiment
    // found and fixed by giving leg_posture_task_weight a nonzero, WEIGHTED
    // - not null-space-subordinate - influence: see the long comment
    // further above about leg_posture_task_weight being non-negotiable).
    // Weighted summation into task1 gave posture continuous, tunable
    // authority over that null-space direction regardless of what task1's
    // OTHER terms wanted; demoting it to strictly-lower-priority gave it
    // authority only within whatever slack task1's own optimal solution
    // left over, which for this specific null-space direction turned out
    // to be far LESS restraining than the old weighted competition, not
    // more - the opposite of the intended effect. Reverted back to the
    // original weighted-sum task1 below. The real distinction between
    // STAND (needs continuous posture authority, weighted-sum works) and
    // TROT_IN_PLACE (posture fighting the ACTIVELY SWINGING leg's own
    // task is what destabilizes it) is per-leg/per-contact-mode, not a
    // single global QP-priority-tier choice - a future fix should
    // probably scale/disable leg_posture's pull specifically for whichever
    // leg is currently swinging (where formulateSwingLegTask() already
    // fully determines that leg's trajectory and posture only adds
    // friction) while keeping it fully weighted for stance legs (where the
    // null-space drift risk actually lives) - not attempted here, this is
    // a real, separate follow-up.
    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask();
    if (std::isfinite(config_.leg_posture_task_weight) && config_.leg_posture_task_weight > 0.0) {
        task1 = task1 + formulateLegJointPostureTask() * config_.leg_posture_task_weight;
    }
    if (std::isfinite(config_.haa_posture_task_weight) && config_.haa_posture_task_weight > 0.0) {
        task1 = task1 + formulateHaaJointPostureTask() * config_.haa_posture_task_weight;
    }
    if (std::isfinite(config_.swing_task_weight) && config_.swing_task_weight > 0.0) {
        task1 = task1 + formulateSwingLegTask() * config_.swing_task_weight;
    }

    HoQp hoQp(task2,
              std::make_shared<HoQp>(task1, std::make_shared<HoQp>(taskJointLimits, std::make_shared<HoQp>(task0))));
    vector_t x_optimal = hoQp.getSolutions();
    return WbcBase::updateCmd(x_optimal);
}

}  // namespace hwbc
}  // namespace megadog
