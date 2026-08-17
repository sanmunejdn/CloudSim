/// @file AssignSpeedZoneOpConfig.cpp
/// @brief AssignSpeedZone 算子配置

// AssignSpeedZone 块参数 schema 与默认 TrajectoryOpDescriptor
#include "AssignSpeedZoneOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeAssignSpeedZoneOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::AssignSpeedZone, "ops/AssignSpeedZone.json");
}

} // namespace trajectory_algo
