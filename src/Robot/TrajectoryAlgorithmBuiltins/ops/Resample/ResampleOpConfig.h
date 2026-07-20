#ifndef TRAJECTORYALGORITHMBUILTINS_RESAMPLEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_RESAMPLEOPCONFIG_H

/// @file ResampleOpConfig.h
/// @brief ResampleOpConfig 接口

// Resample 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeResampleOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_RESAMPLEOPCONFIG_H
