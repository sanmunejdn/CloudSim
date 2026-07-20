/// @file RotateOpConfig.cpp
/// @brief RotateOpConfig 实现

// Rotate 块参数 schema 与默认 TrajectoryOpDescriptor
#include "RotateOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeRotateOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Rotate, "ops/Rotate.json");
}

} // namespace trajectory_algo
