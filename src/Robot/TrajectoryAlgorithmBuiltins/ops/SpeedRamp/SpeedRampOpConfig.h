#ifndef TRAJECTORYALGORITHMBUILTINS_SPEEDRAMPOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_SPEEDRAMPOPCONFIG_H

/// @file SpeedRampOpConfig.h
/// @brief SpeedRampOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeSpeedRampOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_SPEEDRAMPOPCONFIG_H
