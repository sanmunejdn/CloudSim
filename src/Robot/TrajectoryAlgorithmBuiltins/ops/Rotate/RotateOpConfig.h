#ifndef TRAJECTORYALGORITHMBUILTINS_ROTATEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_ROTATEOPCONFIG_H

/// @file RotateOpConfig.h
/// @brief RotateOpConfig 接口

// Rotate 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeRotateOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_ROTATEOPCONFIG_H
