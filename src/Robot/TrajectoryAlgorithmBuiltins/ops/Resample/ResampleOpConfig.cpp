/// @file ResampleOpConfig.cpp
/// @brief Resample 算子配置

// Resample 块参数 schema 与默认 TrajectoryOpDescriptor
#include "ResampleOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeResampleOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::Resample, "ops/Resample.json");
}

} // namespace trajectory_algo
