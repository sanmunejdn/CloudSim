#ifndef TRAJECTORYALGORITHMBUILTINS_TRIMPATHOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_TRIMPATHOPCONFIG_H

/// @file TrimPathOpConfig.h
/// @brief TrimPathOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeTrimPathOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_TRIMPATHOPCONFIG_H
