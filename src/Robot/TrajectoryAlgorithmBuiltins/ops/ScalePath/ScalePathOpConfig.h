#ifndef TRAJECTORYALGORITHMBUILTINS_SCALEPATHOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_SCALEPATHOPCONFIG_H

/// @file ScalePathOpConfig.h
/// @brief ScalePathOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeScalePathOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_SCALEPATHOPCONFIG_H
