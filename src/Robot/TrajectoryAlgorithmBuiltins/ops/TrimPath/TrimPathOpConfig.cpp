/// @file TrimPathOpConfig.cpp
/// @brief TrimPath 算子配置

#include "TrimPathOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeTrimPathOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::TrimPath, "ops/TrimPath.json");
}

} // namespace trajectory_algo
