// Translate 块参数 schema 与默认 TrajectoryOpDescriptor
#include "TranslateOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeTranslateOpConfig()
{
	return makeTrajectoryOpConfig(
		RobotInstruction::TrajectoryOpKind::Translate,
		"ops/Translate.json");
}

} // namespace trajectory_algo
