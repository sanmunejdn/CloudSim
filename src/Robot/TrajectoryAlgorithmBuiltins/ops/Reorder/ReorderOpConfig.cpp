/// @file ReorderOpConfig.cpp
/// @brief Reorder 算子配置

// Reorder 块参数 schema 与默认 TrajectoryOpDescriptor
#include "ReorderOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeReorderOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Reorder, "ops/Reorder.json");
}

} // namespace trajectory_algo
