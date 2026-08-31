#include "megadog_legged_interface/MegadogA1Interface.h"

#include "megadog_legged_interface/package_path.h"

#include <array>
#include <iostream>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>

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

std::unique_ptr<ocs2::StateInputCost> createHaaPositionLimitConstraint(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "haaPositionLimit.";

  // bound<=0 (missing block, or explicitly 0) means "feature off" - see the
  // header doc comment for why this is a distinct, deliberate mechanism from
  // the (exhausted) WBC-level attempts to narrow HAA.
  ocs2::scalar_t bound = 0.0;
  ocs2::SquaredHingePenalty::Config penaltyConfig;
  if (verbose) {
    std::cerr << "\n #### HAA Position Limit (NMPC-level) Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  ocs2::loadData::loadPtreeValue(pt, bound, prefix + "bound", verbose);
  ocs2::loadData::loadPtreeValue(pt, penaltyConfig.mu, prefix + "mu", verbose);
  ocs2::loadData::loadPtreeValue(pt, penaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  if (!(bound > 0.0)) {
    return nullptr;
  }

  // Centroidal state layout is [momentum(6), base pose(6), joints(12)], and
  // task.info's jointNames block fixes joint order to
  // LF_HAA,LF_HFE,LF_KFE,LH_HAA,LH_HFE,LH_KFE,RF_HAA,...,RH_KFE - so the 4
  // HAA joints sit at state indices 12 (LF), 15 (LH), 18 (RF), 21 (RH).
  static constexpr std::array<size_t, 4> kHaaStateIndices{12, 15, 18, 21};

  std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> stateBoxConstraints;
  stateBoxConstraints.reserve(kHaaStateIndices.size());
  for (const size_t index : kHaaStateIndices) {
    ocs2::StateInputSoftBoxConstraint::BoxConstraint box;
    box.index = index;
    box.lowerBound = -bound;
    box.upperBound = bound;
    box.penaltyPtr = std::make_unique<ocs2::SquaredHingePenalty>(penaltyConfig);
    stateBoxConstraints.push_back(std::move(box));
  }

  return std::make_unique<ocs2::StateInputSoftBoxConstraint>(std::move(stateBoxConstraints),
                                                              std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint>{});
}

}  // namespace megadog_legged_interface
