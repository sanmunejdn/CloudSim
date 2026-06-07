// Approach 块参数 schema 与默认 TrajectoryOpDescriptor
#pragma once

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeApproachOpConfig();

} // namespace trajectory_algo
