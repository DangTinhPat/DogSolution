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
