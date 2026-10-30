#ifndef TRAJECTORYALGORITHMBUILTINS_SHIFTSTARTOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_SHIFTSTARTOPCONFIG_H

/// @file ShiftStartOpConfig.h
/// @brief ShiftStartOpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeShiftStartOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_SHIFTSTARTOPCONFIG_H
