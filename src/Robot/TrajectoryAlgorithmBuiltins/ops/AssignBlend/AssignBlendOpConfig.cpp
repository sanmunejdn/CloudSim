// AssignBlend 块参数 schema 与默认 TrajectoryOpDescriptor
#include "AssignBlendOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeAssignBlendOpConfig()
{
	return makeTrajectoryOpConfig(
		RobotInstruction::TrajectoryOpKind::AssignBlend,
		"ops/AssignBlend.json");
}

} // namespace trajectory_algo
