// Approach 块参数 schema 与默认 TrajectoryOpDescriptor
#include "ApproachOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeApproachOpConfig()
{
	return makeTrajectoryOpConfig(
		RobotInstruction::TrajectoryOpKind::Approach,
		"ops/Approach.json");
}

} // namespace trajectory_algo
