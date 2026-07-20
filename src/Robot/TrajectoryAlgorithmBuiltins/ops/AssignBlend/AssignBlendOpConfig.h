#ifndef TRAJECTORYALGORITHMBUILTINS_ASSIGNBLENDOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_ASSIGNBLENDOPCONFIG_H

/// @file AssignBlendOpConfig.h
/// @brief AssignBlendOpConfig 接口

// AssignBlend 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeAssignBlendOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_ASSIGNBLENDOPCONFIG_H
