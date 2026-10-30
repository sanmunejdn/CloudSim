#ifndef TRAJECTORYALGORITHMBUILTINS_SPIKEFILTEROPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_SPIKEFILTEROPCONFIG_H

/// @file SpikeFilterOpConfig.h
/// @brief SpikeFilterOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeSpikeFilterOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_SPIKEFILTEROPCONFIG_H
