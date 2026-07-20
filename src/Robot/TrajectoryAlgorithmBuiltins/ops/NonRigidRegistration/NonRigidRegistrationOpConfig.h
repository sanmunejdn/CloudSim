#ifndef TRAJECTORYALGORITHMBUILTINS_NONRIGIDREGISTRATIONOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_NONRIGIDREGISTRATIONOPCONFIG_H

/// @file NonRigidRegistrationOpConfig.h
/// @brief NonRigidRegistrationOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeNonRigidRegistrationOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_NONRIGIDREGISTRATIONOPCONFIG_H
