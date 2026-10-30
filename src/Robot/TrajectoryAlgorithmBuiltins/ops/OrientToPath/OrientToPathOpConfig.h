#ifndef TRAJECTORYALGORITHMBUILTINS_ORIENTTOPATHOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_ORIENTTOPATHOPCONFIG_H

/// @file OrientToPathOpConfig.h
/// @brief OrientToPathOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeOrientToPathOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_ORIENTTOPATHOPCONFIG_H
