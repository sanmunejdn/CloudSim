/// @file DeleteOpConfig.cpp
/// @brief DeleteOpConfig 实现

// Delete 块参数 schema 与默认 TrajectoryOpDescriptor
#include "DeleteOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeDeleteOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Delete, "ops/Delete.json");
}

} // namespace trajectory_algo
