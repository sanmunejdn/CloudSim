/// @file ScalePathOpConfig.cpp
/// @brief ScalePath 算子配置

#include "ScalePathOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeScalePathOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::ScalePath, "ops/ScalePath.json");
}

} // namespace trajectory_algo
