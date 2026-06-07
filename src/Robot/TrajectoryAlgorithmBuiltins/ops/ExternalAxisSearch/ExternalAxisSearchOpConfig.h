// ExternalAxisSearch 块参数 schema 与默认 TrajectoryOpDescriptor
#pragma once

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeExternalAxisSearchOpConfig();

} // namespace trajectory_algo
