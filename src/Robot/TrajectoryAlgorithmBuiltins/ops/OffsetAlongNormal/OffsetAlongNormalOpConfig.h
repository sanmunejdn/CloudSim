#ifndef TRAJECTORYALGORITHMBUILTINS_OFFSETALONGNORMALOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_OFFSETALONGNORMALOPCONFIG_H

/// @file OffsetAlongNormalOpConfig.h
/// @brief OffsetAlongNormalOpConfig 接口

// OffsetAlongNormal 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeOffsetAlongNormalOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_OFFSETALONGNORMALOPCONFIG_H
