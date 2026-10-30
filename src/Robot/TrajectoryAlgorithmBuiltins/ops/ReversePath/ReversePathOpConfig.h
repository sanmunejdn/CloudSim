#ifndef TRAJECTORYALGORITHMBUILTINS_REVERSEPATHOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_REVERSEPATHOPCONFIG_H

/// @file ReversePathOpConfig.h
/// @brief ReversePathOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeReversePathOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_REVERSEPATHOPCONFIG_H
