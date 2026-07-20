#ifndef TRAJECTORYALGORITHMBUILTINS_MIRROROPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_MIRROROPCONFIG_H

/// @file MirrorOpConfig.h
/// @brief MirrorOpConfig 接口

// Mirror 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeMirrorOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_MIRROROPCONFIG_H
