#ifndef TRAJECTORYALGORITHMBUILTINS_CLAMPTOPLANEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_CLAMPTOPLANEOPCONFIG_H

/// @file ClampToPlaneOpConfig.h
/// @brief ClampToPlaneOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeClampToPlaneOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_CLAMPTOPLANEOPCONFIG_H
