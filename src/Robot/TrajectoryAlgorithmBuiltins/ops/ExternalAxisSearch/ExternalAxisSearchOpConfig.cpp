/// @file ExternalAxisSearchOpConfig.cpp
/// @brief ExternalAxisSearch 算子配置

// ExternalAxisSearch 块参数 schema 与默认 TrajectoryOpDescriptor
#include "ExternalAxisSearchOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeExternalAxisSearchOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::ExternalAxisSearch,
								  "ops/ExternalAxisSearch.json");
}

} // namespace trajectory_algo
