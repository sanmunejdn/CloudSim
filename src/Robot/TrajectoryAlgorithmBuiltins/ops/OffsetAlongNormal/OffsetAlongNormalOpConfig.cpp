/// @file OffsetAlongNormalOpConfig.cpp
/// @brief OffsetAlongNormal 算子配置

// OffsetAlongNormal 块参数 schema 与默认 TrajectoryOpDescriptor
#include "OffsetAlongNormalOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeOffsetAlongNormalOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::OffsetAlongNormal, "ops/OffsetAlongNormal.json");
}

} // namespace trajectory_algo
