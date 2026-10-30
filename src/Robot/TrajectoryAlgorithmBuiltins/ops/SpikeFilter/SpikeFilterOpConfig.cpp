/// @file SpikeFilterOpConfig.cpp
/// @brief SpikeFilter 算子配置

#include "SpikeFilterOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeSpikeFilterOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::SpikeFilter, "ops/SpikeFilter.json");
}

} // namespace trajectory_algo
