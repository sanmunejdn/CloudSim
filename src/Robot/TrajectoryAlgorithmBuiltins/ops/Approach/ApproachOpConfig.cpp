/// @file ApproachOpConfig.cpp
/// @brief ApproachOpConfig 实现

// Approach 块参数 schema 与默认 TrajectoryOpDescriptor
#include "ApproachOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeApproachOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Approach, "ops/Approach.json");
}

} // namespace trajectory_algo
