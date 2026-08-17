/// @file MirrorOpConfig.cpp
/// @brief Mirror 算子配置

// Mirror 块参数 schema 与默认 TrajectoryOpDescriptor
#include "MirrorOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeMirrorOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Mirror, "ops/Mirror.json");
}

} // namespace trajectory_algo
