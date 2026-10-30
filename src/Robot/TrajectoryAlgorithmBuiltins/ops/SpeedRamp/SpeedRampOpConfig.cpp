/// @file SpeedRampOpConfig.cpp
/// @brief SpeedRamp 算子配置

#include "SpeedRampOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeSpeedRampOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::SpeedRamp, "ops/SpeedRamp.json");
}

} // namespace trajectory_algo
