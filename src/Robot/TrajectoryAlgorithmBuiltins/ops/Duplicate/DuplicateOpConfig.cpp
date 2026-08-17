/// @file DuplicateOpConfig.cpp
/// @brief Duplicate 算子配置

// Duplicate 块参数 schema 与默认 TrajectoryOpDescriptor
#include "DuplicateOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeDuplicateOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Duplicate, "ops/Duplicate.json");
}

} // namespace trajectory_algo
