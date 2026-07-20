#ifndef TRAJECTORYALGORITHMBUILTINS_DELETEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_DELETEOPCONFIG_H

/// @file DeleteOpConfig.h
/// @brief DeleteOpConfig 接口

// Delete 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeDeleteOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_DELETEOPCONFIG_H
