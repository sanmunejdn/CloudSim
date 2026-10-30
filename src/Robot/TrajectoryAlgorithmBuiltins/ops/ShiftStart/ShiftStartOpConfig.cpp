/// @file ShiftStartOpConfig.cpp
/// @brief ShiftStart 算子配置

#include "ShiftStartOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeShiftStartOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::ShiftStart, "ops/ShiftStart.json");
}

} // namespace trajectory_algo
