#ifndef TRAJECTORYALGORITHMBUILTINS_REACHABILITYFILTEROPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_REACHABILITYFILTEROPCONFIG_H

/// @file ReachabilityFilterOpConfig.h
/// @brief ReachabilityFilterOpConfig 接口

// ReachabilityFilter 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeReachabilityFilterOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_REACHABILITYFILTEROPCONFIG_H
