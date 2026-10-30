#ifndef TRAJECTORYALGORITHMBUILTINS_INSERTMIDOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_INSERTMIDOPCONFIG_H

/// @file InsertMidOpConfig.h
/// @brief InsertMidOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeInsertMidOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_INSERTMIDOPCONFIG_H
