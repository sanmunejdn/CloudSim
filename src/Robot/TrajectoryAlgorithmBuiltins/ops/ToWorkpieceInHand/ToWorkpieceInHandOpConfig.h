#ifndef TRAJECTORYALGORITHMBUILTINS_TOWORKPIECEINHANDOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_TOWORKPIECEINHANDOPCONFIG_H

/// @file ToWorkpieceInHandOpConfig.h
/// @brief ToWorkpieceInHandOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeToWorkpieceInHandOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_TOWORKPIECEINHANDOPCONFIG_H
