#include <gtest/gtest.h>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/utils.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_sqp/SqpMpc.h>

#include "megadog_legged_interface/MegadogA1Interface.h"

#include <array>
#include <vector>

using namespace ocs2;
using namespace ocs2::legged_robot;

namespace {
// A1 has no arm: stock ocs2_legged_robot's SRBD-with-joints layout is
// exactly 6 (centroidal momentum) + 6 (base pose) + 12 (leg joints) state,
// 12 (contact forces) + 12 (leg joint velocities) input. This constant pins
// that against ever accidentally reintroducing an arm-sized (30/30) layout.
constexpr size_t kExpectedDim = 24;
}  // namespace

TEST(MegadogA1LeggedInterface, buildsWithExpectedDimensions) {
  const auto interface = megadog_legged_interface::createInterface();

  EXPECT_EQ(interface->getCentroidalModelInfo().stateDim, kExpectedDim);
  EXPECT_EQ(interface->getCentroidalModelInfo().inputDim, kExpectedDim);
  EXPECT_EQ(interface->getCentroidalModelInfo().numThreeDofContacts, 4u);
  EXPECT_EQ(interface->getCentroidalModelInfo().actuatedDofNum, 12u);

  ASSERT_EQ(interface->getInitialState().size(), static_cast<long>(kExpectedDim));
  for (long i = 0; i < interface->getInitialState().size(); ++i) {
    EXPECT_TRUE(std::isfinite(interface->getInitialState()(i)));
  }
}

TEST(MegadogA1LeggedInterface, modelSettingsUseMegadogJointAndOcs2ContactOrder) {
  const auto interface = megadog_legged_interface::createInterface();

  const std::vector<std::string> expectedJointNames{
      "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
      "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  const std::vector<std::string> expectedContactNames{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

  EXPECT_EQ(interface->modelSettings().jointNames, expectedJointNames);
  EXPECT_EQ(interface->modelSettings().contactNames3DoF, expectedContactNames);

  const vector_t jointState = interface->getInitialState().tail(interface->getCentroidalModelInfo().actuatedDofNum);
  ASSERT_EQ(jointState.size(), 12);
  const std::array<double, 12> expectedJointState{
      -0.451, 0.487400, -1.199164,
      -0.451, 0.487400, -1.199164,
       0.451, 0.487400, -1.199164,
       0.451, 0.487400, -1.199164};
  for (long i = 0; i < jointState.size(); ++i) {
    EXPECT_NEAR(jointState(i), expectedJointState[static_cast<size_t>(i)], 1e-9);
  }
}

TEST(MegadogA1LeggedInterface, sqpMpcSolvesStandStillToAFiniteInputTrajectory) {
  const auto interface = megadog_legged_interface::createInterface();

  SqpMpc mpc(interface->mpcSettings(), interface->sqpSettings(), interface->getOptimalControlProblem(),
             interface->getInitializer());
  // Without this, the solver falls back to a no-op base ReferenceManager and
  // SwitchedModelReferenceManager::modifyReferences() (which populates the
  // SwingTrajectoryPlanner) never runs, leaving it with empty per-leg
  // trajectories - crashes on the first constraint evaluation.
  mpc.getSolverPtr()->setReferenceManager(interface->getReferenceManagerPtr());

  const vector_t initState = interface->getInitialState();
  const contact_flag_t allStance{true, true, true, true};
  const vector_t desiredInput = weightCompensatingInput(interface->getCentroidalModelInfo(), allStance);

  // Stand-still target: hold the initial state/input for the whole horizon.
  const scalar_t horizon = interface->mpcSettings().timeHorizon_;
  const TargetTrajectories targetTrajectories({0.0, horizon}, {initState, initState}, {desiredInput, desiredInput});
  interface->getReferenceManagerPtr()->setTargetTrajectories(targetTrajectories);

  ASSERT_TRUE(mpc.run(0.0, initState));

  PrimalSolution primalSolution;
  mpc.getSolverPtr()->getPrimalSolution(horizon, &primalSolution);

  ASSERT_FALSE(primalSolution.timeTrajectory_.empty());
  ASSERT_FALSE(primalSolution.inputTrajectory_.empty());
  ASSERT_FALSE(primalSolution.stateTrajectory_.empty());

  for (const auto& input : primalSolution.inputTrajectory_) {
    ASSERT_EQ(input.size(), static_cast<long>(kExpectedDim));
    for (long i = 0; i < input.size(); ++i) {
      EXPECT_TRUE(std::isfinite(input(i)));
    }
  }
  for (const auto& state : primalSolution.stateTrajectory_) {
    ASSERT_EQ(state.size(), static_cast<long>(kExpectedDim));
    for (long i = 0; i < state.size(); ++i) {
      EXPECT_TRUE(std::isfinite(state(i)));
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
