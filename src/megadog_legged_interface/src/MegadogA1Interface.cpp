#include "megadog_legged_interface/MegadogA1Interface.h"

#include "megadog_legged_interface/package_path.h"

namespace megadog_legged_interface {

std::string getUrdfPath() {
  return getGeneratedUrdfPath();
}

std::string getConfigPath(const std::string& fileName) {
  return getSourcePath() + "/config/" + fileName;
}

std::unique_ptr<ocs2::legged_robot::LeggedRobotInterface> createInterface(bool useHardFrictionConeConstraint) {
  return std::make_unique<ocs2::legged_robot::LeggedRobotInterface>(getConfigPath("task.info"), getUrdfPath(),
                                                                     getConfigPath("reference.info"),
                                                                     useHardFrictionConeConstraint);
}

}  // namespace megadog_legged_interface
