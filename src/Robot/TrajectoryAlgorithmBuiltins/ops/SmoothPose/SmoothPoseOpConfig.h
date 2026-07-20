#ifndef TRAJECTORYALGORITHMBUILTINS_SMOOTHPOSEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_SMOOTHPOSEOPCONFIG_H

/// @file SmoothPoseOpConfig.h
/// @brief SmoothPoseOpConfig 接口

// SmoothPose 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeSmoothPoseOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_SMOOTHPOSEOPCONFIG_H
