/// @file InsertMidOpConfig.cpp
/// @brief InsertMid 算子配置

#include "InsertMidOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeInsertMidOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::InsertMid, "ops/InsertMid.json");
}

} // namespace trajectory_algo
