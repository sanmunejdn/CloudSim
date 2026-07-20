#ifndef TRAJECTORYALGORITHMBUILTINS_DUPLICATEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_DUPLICATEOPCONFIG_H

/// @file DuplicateOpConfig.h
/// @brief DuplicateOpConfig 接口

// Duplicate 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeDuplicateOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_DUPLICATEOPCONFIG_H
