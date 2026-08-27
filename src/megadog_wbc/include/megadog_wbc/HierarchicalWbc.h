#ifndef CONTROLLER_CONTROL_HIERARCHICALWBC_H
#define CONTROLLER_CONTROL_HIERARCHICALWBC_H

// Ported from skywoodsz/qm_control's qm_wbc::HierarchicalWbc, arm task/branch
// removed (babyDog has no arm - the original's "hold arm at nominal pose
// for the first 10s" warm-up branch does not apply). See WbcBase.h and
// /home/dvt/.claude/plans/purrfect-imagining-hartmanis.md (Milestone 3).

#include "megadog_wbc/WbcBase.h"

namespace megadog
{
namespace hwbc
{

// 4-level task priority, highest first:
//   0 (hard):        floating-base EOM + torque limits + no-contact-motion + friction cone
//   1 (joint limit): joint position-limit avoidance (safety preference, not physics -
//                    see WbcBase::formulateJointLimitsTask - kept out of level 0 so it
//                    never fights the true physics constraints above it)
//   2 (mid):         base height/angular tracking + swing-leg tracking
//   3 (low):         contact-force tracking + base xy linear tracking
class HierarchicalWbc : public WbcBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HierarchicalWbc(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                    const PinocchioEndEffectorKinematics& eeKinematics, HierarchicalWbcConfig config = {});

    vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                    size_t mode, scalar_t period, scalar_t time) override;
};

}  // namespace hwbc
}  // namespace megadog

#endif  // CONTROLLER_CONTROL_HIERARCHICALWBC_H
