/// @file OrientToPathOpConfig.cpp
/// @brief OrientToPath 算子配置

#include "OrientToPathOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeOrientToPathOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::OrientToPath, "ops/OrientToPath.json");
}

} // namespace trajectory_algo
