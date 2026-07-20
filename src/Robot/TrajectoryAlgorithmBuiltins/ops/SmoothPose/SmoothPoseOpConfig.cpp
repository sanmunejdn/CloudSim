/// @file SmoothPoseOpConfig.cpp
/// @brief SmoothPoseOpConfig 实现

// SmoothPose 块参数 schema 与默认 TrajectoryOpDescriptor
#include "SmoothPoseOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeSmoothPoseOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::SmoothPose, "ops/SmoothPose.json");
}

} // namespace trajectory_algo
