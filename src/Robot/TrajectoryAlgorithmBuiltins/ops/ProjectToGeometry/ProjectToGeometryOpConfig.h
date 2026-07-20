#ifndef TRAJECTORYALGORITHMBUILTINS_PROJECTTOGEOMETRYOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_PROJECTTOGEOMETRYOPCONFIG_H

/// @file ProjectToGeometryOpConfig.h
/// @brief ProjectToGeometryOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeProjectToGeometryOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_PROJECTTOGEOMETRYOPCONFIG_H
