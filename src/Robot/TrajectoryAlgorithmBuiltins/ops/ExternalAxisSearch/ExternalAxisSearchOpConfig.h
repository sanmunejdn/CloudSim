#ifndef TRAJECTORYALGORITHMBUILTINS_EXTERNALAXISSEARCHOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_EXTERNALAXISSEARCHOPCONFIG_H

/// @file ExternalAxisSearchOpConfig.h
/// @brief ExternalAxisSearchOpConfig 接口

// ExternalAxisSearch 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeExternalAxisSearchOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_EXTERNALAXISSEARCHOPCONFIG_H
