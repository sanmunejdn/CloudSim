#ifndef TRAJECTORYALGORITHMBUILTINS_ASSIGNSPEEDZONEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_ASSIGNSPEEDZONEOPCONFIG_H

/// @file AssignSpeedZoneOpConfig.h
/// @brief AssignSpeedZoneOpConfig 接口

// AssignSpeedZone 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeAssignSpeedZoneOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_ASSIGNSPEEDZONEOPCONFIG_H
