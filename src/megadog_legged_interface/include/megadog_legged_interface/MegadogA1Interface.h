#pragma once

#include <memory>
#include <string>

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

// Thin wrapper standing up stock ocs2::legged_robot::LeggedRobotInterface
// against A1's own URDF/task/reference files - same pattern as babyDog's
// babydog_legged_interface (see /home/dvt/babyDog/src/babydog_legged_interface).
// legged_control's own `legged_interface::LeggedInterface` is itself just a
// thin extension of this exact stock ocs2_legged_robot class (self-collision
// + a custom initializer) - deferred here, see the megaDog port plan's
// Milestone 2 note. A1 has no arm, so its state (24 = 6 centroidal momentum +
// 6 base pose + 12 leg joints) and input (24 = 12 contact forces + 12 leg
// joint velocities) are exactly the stock ocs2_legged_robot layout.
namespace megadog_legged_interface {

// Absolute path to the plain (non-xacro) URDF generated at build time from
// megadog_description's urdf/robot.xacro (see CMakeLists.txt's xacro custom
// command).
std::string getUrdfPath();

// Absolute path to a file under this package's config/ directory.
std::string getConfigPath(const std::string& fileName);

// Builds the interface against config/task.info + the generated URDF +
// config/reference.info.
std::unique_ptr<ocs2::legged_robot::LeggedRobotInterface> createInterface(bool useHardFrictionConeConstraint = false);

// Builds a genuine NMPC-level (not WBC-level) soft box constraint on the 4
// HAA (hip ab/adduction) centroidal state indices, read from taskFile's
// `haaPositionLimit { bound <rad>  mu <>  delta <rad> }` block. Unlike a
// WBC-layer task/constraint - which only sees the trajectory one control
// tick after the NMPC has already committed to it - this is added directly
// to the OptimalControlProblem handed to SqpMpc, so it is evaluated inside
// the SQP's own horizon at every transcription node, letting the optimizer
// bend the trajectory before an overshoot rather than reacting to one
// downstream. Uses ocs2::StateInputSoftBoxConstraint (a stock, unmodified
// OCS2 core class, same StateInputCost base as everything already in the
// problem) with a SquaredHingePenalty per HAA joint - exactly zero cost
// until the state gets within `delta` of `bound`, so it doesn't compete
// with base/foot-placement tracking anywhere except right at the safety
// edge. See MegadogWbcRuntime.cpp's construction site for how this is
// spliced into a local copy of the interface's OptimalControlProblem
// (LeggedRobotInterface::getOptimalControlProblem() returns only a const&,
// and the class is `final` with a private problemPtr_, so it cannot be
// mutated in place - OptimalControlProblem's own public copy constructor,
// and SqpMpc's constructor already cloning whatever ControlProblem it's
// given, are what make copy-then-splice-then-construct safe here without
// touching vendored ocs2_legged_robot code).
// Returns nullptr if `bound` is <= 0 in taskFile (feature-flag: constraint
// left out entirely rather than added with a degenerate bound).
std::unique_ptr<ocs2::StateInputCost> createHaaPositionLimitConstraint(const std::string& taskFile, bool verbose);

}  // namespace megadog_legged_interface
