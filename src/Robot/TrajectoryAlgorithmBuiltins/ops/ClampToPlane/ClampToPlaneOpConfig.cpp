/// @file ClampToPlaneOpConfig.cpp
/// @brief ClampToPlane 算子配置

#include "ClampToPlaneOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeClampToPlaneOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::ClampToPlane, "ops/ClampToPlane.json");
}

} // namespace trajectory_algo
