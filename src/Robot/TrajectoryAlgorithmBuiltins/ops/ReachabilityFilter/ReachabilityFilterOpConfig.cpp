/// @file ReachabilityFilterOpConfig.cpp
/// @brief ReachabilityFilter 算子配置

// ReachabilityFilter 块参数 schema 与默认 TrajectoryOpDescriptor
#include "ReachabilityFilterOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeReachabilityFilterOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::ReachabilityFilter,
								  "ops/ReachabilityFilter.json");
}

} // namespace trajectory_algo
