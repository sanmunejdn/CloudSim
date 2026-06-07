// Weave 块参数 schema 与默认 TrajectoryOpDescriptor
#include "WeaveOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeWeaveOpConfig()
{
	return makeTrajectoryOpConfig(
		RobotInstruction::TrajectoryOpKind::Weave,
		"ops/Weave.json");
}

} // namespace trajectory_algo
