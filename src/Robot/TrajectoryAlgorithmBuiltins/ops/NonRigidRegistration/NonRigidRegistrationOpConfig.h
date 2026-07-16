#pragma once

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeNonRigidRegistrationOpConfig();

} // namespace trajectory_algo
