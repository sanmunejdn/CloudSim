/// @file ReversePathOpConfig.cpp
/// @brief ReversePath 算子配置

#include "ReversePathOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeReversePathOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::ReversePath, "ops/ReversePath.json");
}

} // namespace trajectory_algo
