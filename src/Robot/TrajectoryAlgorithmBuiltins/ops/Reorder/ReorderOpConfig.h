#ifndef TRAJECTORYALGORITHMBUILTINS_REORDEROPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_REORDEROPCONFIG_H

/// @file ReorderOpConfig.h
/// @brief ReorderOpConfig 接口

// Reorder 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeReorderOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_REORDEROPCONFIG_H
