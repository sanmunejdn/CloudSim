// OffsetLateral 块参数 schema 与默认 TrajectoryOpDescriptor
#include "OffsetLateralOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeOffsetLateralOpConfig()
{
	return makeTrajectoryOpConfig(
		RobotInstruction::TrajectoryOpKind::OffsetLateral,
		"ops/OffsetLateral.json");
}

} // namespace trajectory_algo
