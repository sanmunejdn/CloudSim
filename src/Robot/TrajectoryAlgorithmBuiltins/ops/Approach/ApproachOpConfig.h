#ifndef TRAJECTORYALGORITHMBUILTINS_APPROACHOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_APPROACHOPCONFIG_H

/// @file ApproachOpConfig.h
/// @brief ApproachOpConfig 接口

// Approach 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeApproachOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_APPROACHOPCONFIG_H
