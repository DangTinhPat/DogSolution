#pragma once

#include <memory>
#include <string>

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

}  // namespace megadog_legged_interface
