#include <gtest/gtest.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/common/utils.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include "megadog_legged_interface/MegadogA1Interface.h"
#include "megadog_wbc/HierarchicalWbc.h"

using namespace ocs2;
using namespace ocs2::legged_robot;

namespace
{
// Mirrors megadog_legged_interface's test: pins the SRBD-with-joints layout
// (no arm) so this test breaks loudly if that ever changes.
constexpr size_t kExpectedStateDim = 24;

class InspectableWbc final : public megadog::hwbc::WbcBase
{
public:
    using megadog::hwbc::WbcBase::WbcBase;

    megadog::hwbc::Task torqueLimitsTask(const vector_t& stateDesired,
                                            const vector_t& inputDesired,
                                            const vector_t& rbdStateMeasured,
                                            const size_t mode,
                                            const scalar_t period,
                                            const scalar_t time)
    {
        megadog::hwbc::WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
        return formulateTorqueLimitsTask();
    }

    megadog::hwbc::Task haaJointPostureTask(const vector_t& stateDesired,
                                            const vector_t& inputDesired,
                                            const vector_t& rbdStateMeasured,
                                            const size_t mode,
                                            const scalar_t period,
                                            const scalar_t time)
    {
        megadog::hwbc::WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
        return formulateHaaJointPostureTask();
    }
};

vector_t standStillRbdState(const vector_t& initState, const CentroidalModelInfo& info)
{
    const size_t nq = info.generalizedCoordinatesNum;
    vector_t rbdStateMeasured = vector_t::Zero(2 * nq);
    rbdStateMeasured.segment<3>(3) = initState.segment<3>(6);
    rbdStateMeasured.segment<3>(0) = initState.segment<3>(9);
    rbdStateMeasured.segment(6, info.actuatedDofNum) = initState.tail(info.actuatedDofNum);
    return rbdStateMeasured;
}

void expectWbcModeFiniteAtStandStill(const size_t mode)
{
    const auto interface = megadog_legged_interface::createInterface();
    const auto& info = interface->getCentroidalModelInfo();
    ASSERT_EQ(info.stateDim, kExpectedStateDim);

    CentroidalModelPinocchioMapping mapping(info);
    PinocchioEndEffectorKinematics eeKinematics(interface->getPinocchioInterface(), mapping,
                                                interface->modelSettings().contactNames3DoF);

    megadog::hwbc::HierarchicalWbc wbc(interface->getPinocchioInterface(), info, eeKinematics);

    const vector_t initState = interface->getInitialState();
    const contact_flag_t contacts = modeNumber2StanceLeg(mode);
    const vector_t desiredInput = weightCompensatingInput(info, contacts);

    const size_t nq = info.generalizedCoordinatesNum;
    const vector_t rbdStateMeasured = standStillRbdState(initState, info);

    const scalar_t period = 0.001;
    const scalar_t time = 20.0;
    const vector_t cmd = wbc.update(initState, desiredInput, rbdStateMeasured, mode, period, time);

    const size_t numDecisionVars = nq + 3 * info.numThreeDofContacts;
    ASSERT_EQ(cmd.size(), static_cast<long>(numDecisionVars + info.actuatedDofNum));
    for (long i = 0; i < cmd.size(); ++i) {
        ASSERT_TRUE(std::isfinite(cmd(i))) << "cmd(" << i << ") is not finite for mode " << mode;
    }

    const vector_t vDotAll = cmd.head(nq);
    const vector_t forces = cmd.segment(nq, 3 * info.numThreeDofContacts);
    const vector_t torque = cmd.tail(info.actuatedDofNum);

    auto pinocchioInterface = interface->getPinocchioInterface();
    const auto& model = pinocchioInterface.getModel();
    auto& data = pinocchioInterface.getData();

    vector_t q = vector_t::Zero(nq);
    q.head<3>() = rbdStateMeasured.segment<3>(3);
    q.segment<3>(3) = rbdStateMeasured.segment<3>(0);
    q.tail(info.actuatedDofNum) = rbdStateMeasured.segment(6, info.actuatedDofNum);
    const vector_t v = vector_t::Zero(nq);

    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, q, v);

    matrix_t jStacked(3 * info.numThreeDofContacts, nq);
    for (size_t i = 0; i < info.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, nq);
        pinocchio::getFrameJacobian(model, data, info.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        jStacked.block(3 * i, 0, 3, nq) = jac.topRows<3>();
    }

    const matrix_t Mb = data.M.topRows(6);
    const vector_t hb = data.nle.topRows(6);
    const matrix_t JbT = jStacked.transpose().topRows(6);
    const vector_t residual = Mb * vDotAll - JbT * forces + hb;
    // qpOASES solves task0's EOM equality as a least-squares objective, not
    // an exact analytic solve (see HoQp::buildHMatrix) - so "residual ~ 0"
    // means "at the solver's own numerical precision", not machine epsilon.
    // 1e-4 is still ~5 orders of magnitude below the 9.1 N.m torque scale
    // this system operates at, and ~4 orders of magnitude below the ~1.0
    // residual magnitude a genuinely broken/infeasible solve produces (see
    // StateTrot's runtime eom_residual diagnostic in a failing sim run) - so
    // this stays a real correctness check, just not a machine-precision one.
    for (long i = 0; i < residual.size(); ++i) {
        EXPECT_NEAR(residual(i), 0.0, 1e-4)
            << "floating-base EOM residual index " << i << " for mode " << mode;
    }

    for (long i = 0; i < torque.size(); ++i) {
        EXPECT_TRUE(std::isfinite(torque(i))) << "torque(" << i << ") mode " << mode;
        // Simulation debug limit from const.xacro/task.info.
        EXPECT_LE(std::abs(torque(i)), 80.0) << "torque(" << i << ") mode " << mode;
    }
}
}  // namespace

// Builds a real HierarchicalWbc against A1's OCS2 interface (Milestone 2)
// and independently recomputes the floating-base equation-of-motion residual
// (Mb*vDot - Jb^T*F + hb) from the QP's own decision-vector output, using a
// fresh Pinocchio evaluation rather than anything internal to WbcBase - this
// proves the hierarchical QP's hard level-0 constraint is actually satisfied,
// not just that update() returned without crashing.
TEST(HierarchicalWbc, satisfiesFloatingBaseEomAtStandStill)
{
    expectWbcModeFiniteAtStandStill(ModeNumber::STANCE);
}

TEST(HierarchicalWbc, satisfiesFloatingBaseEomInDiagonalSupportAtStandStill)
{
    expectWbcModeFiniteAtStandStill(ModeNumber::LF_RH);
    expectWbcModeFiniteAtStandStill(ModeNumber::RF_LH);
}

TEST(HierarchicalWbc, torqueLimitTaskAppliesConfiguredLimitsToBothDirections)
{
    const auto interface = megadog_legged_interface::createInterface();
    const auto& info = interface->getCentroidalModelInfo();
    ASSERT_EQ(info.stateDim, kExpectedStateDim);

    CentroidalModelPinocchioMapping mapping(info);
    PinocchioEndEffectorKinematics eeKinematics(interface->getPinocchioInterface(), mapping,
                                                interface->modelSettings().contactNames3DoF);

    megadog::hwbc::HierarchicalWbcConfig config;
    config.leg_torque_limits_nm = {1.25, 2.5, 3.75};
    InspectableWbc wbc(interface->getPinocchioInterface(), info, eeKinematics, config);

    const vector_t initState = interface->getInitialState();
    const contact_flag_t contacts = modeNumber2StanceLeg(ModeNumber::STANCE);
    const vector_t desiredInput = weightCompensatingInput(info, contacts);
    const vector_t rbdStateMeasured = standStillRbdState(initState, info);
    const megadog::hwbc::Task task =
        wbc.torqueLimitsTask(initState, desiredInput, rbdStateMeasured, ModeNumber::STANCE, 0.001, 20.0);

    ASSERT_EQ(task.d_.rows(), static_cast<long>(2 * info.actuatedDofNum));
    ASSERT_EQ(task.f_.rows(), static_cast<long>(2 * info.actuatedDofNum));

    const size_t nq = info.generalizedCoordinatesNum;
    auto pinocchioInterface = interface->getPinocchioInterface();
    const auto& model = pinocchioInterface.getModel();
    auto& data = pinocchioInterface.getData();

    vector_t q = vector_t::Zero(nq);
    q.head<3>() = rbdStateMeasured.segment<3>(3);
    q.segment<3>(3) = rbdStateMeasured.segment<3>(0);
    q.tail(info.actuatedDofNum) = rbdStateMeasured.segment(6, info.actuatedDofNum);
    const vector_t v = vector_t::Zero(nq);

    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, q, v);

    const vector_t hj = data.nle.bottomRows(info.actuatedDofNum);
    vector_t expectedLimits(info.actuatedDofNum);
    for (size_t leg = 0; leg < info.numThreeDofContacts; ++leg) {
        expectedLimits.segment<3>(3 * leg) << 1.25, 2.5, 3.75;
    }

    for (long i = 0; i < static_cast<long>(info.actuatedDofNum); ++i) {
        EXPECT_NEAR(task.f_(i), expectedLimits(i) - hj(i), 1e-9)
            << "upper torque-limit RHS index " << i;
        EXPECT_NEAR(task.f_(info.actuatedDofNum + i), expectedLimits(i) + hj(i), 1e-9)
            << "lower torque-limit RHS index " << i;
    }
}

TEST(HierarchicalWbc, haaJointPostureTaskPullsAbadJointsTowardNominal)
{
    const auto interface = megadog_legged_interface::createInterface();
    const auto& info = interface->getCentroidalModelInfo();
    ASSERT_EQ(info.stateDim, kExpectedStateDim);

    CentroidalModelPinocchioMapping mapping(info);
    PinocchioEndEffectorKinematics eeKinematics(interface->getPinocchioInterface(), mapping,
                                                interface->modelSettings().contactNames3DoF);

    megadog::hwbc::HierarchicalWbcConfig config;
    config.haa_posture_kp = 10.0;
    config.haa_posture_kd = 0.0;
    InspectableWbc wbc(interface->getPinocchioInterface(), info, eeKinematics, config);

    // Offset the measured HAA joints by 0.2 rad further into the splay
    // direction from whatever reference.info/task.info's current nominal is
    // (read via initState below, not hardcoded here) - keeps this test
    // meaningful regardless of the nominal HAA value in effect.
    const vector_t initState = interface->getInitialState();
    vector_t rbdStateMeasured = standStillRbdState(initState, info);
    rbdStateMeasured(6 + 0) = initState(12 + 0) - 0.20;
    rbdStateMeasured(6 + 3) = initState(12 + 3) - 0.20;
    rbdStateMeasured(6 + 6) = initState(12 + 6) + 0.20;
    rbdStateMeasured(6 + 9) = initState(12 + 9) + 0.20;

    const vector_t desiredInput = vector_t::Zero(info.inputDim);
    const megadog::hwbc::Task task =
        wbc.haaJointPostureTask(initState, desiredInput, rbdStateMeasured, ModeNumber::STANCE, 0.001, 20.0);

    ASSERT_EQ(task.a_.rows(), 4);
    ASSERT_EQ(task.b_.rows(), 4);

    const std::array<size_t, 4> haaJointIndices{0, 3, 6, 9};
    for (size_t row = 0; row < haaJointIndices.size(); ++row) {
        for (long col = 0; col < task.a_.cols(); ++col) {
            const double expected = col == static_cast<long>(6 + haaJointIndices[row]) ? 1.0 : 0.0;
            EXPECT_NEAR(task.a_(static_cast<long>(row), col), expected, 1e-12)
                << "row " << row << " col " << col;
        }
    }

    EXPECT_NEAR(task.b_(0), 2.0, 1e-12);
    EXPECT_NEAR(task.b_(1), 2.0, 1e-12);
    EXPECT_NEAR(task.b_(2), -2.0, 1e-12);
    EXPECT_NEAR(task.b_(3), -2.0, 1e-12);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
