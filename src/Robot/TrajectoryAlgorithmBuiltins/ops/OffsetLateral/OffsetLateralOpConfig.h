#ifndef TRAJECTORYALGORITHMBUILTINS_OFFSETLATERALOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_OFFSETLATERALOPCONFIG_H

/// @file OffsetLateralOpConfig.h
/// @brief OffsetLateralOpConfig 接口

// OffsetLateral 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeOffsetLateralOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_OFFSETLATERALOPCONFIG_H
