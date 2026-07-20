/// @file RetractOpConfig.cpp
/// @brief RetractOpConfig 实现

// Retract 块参数 schema 与默认 TrajectoryOpDescriptor
#include "RetractOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeRetractOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Retract, "ops/Retract.json");
}

} // namespace trajectory_algo
