#include "ProjectToGeometryOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

std::unique_ptr<IOpParamConfig> makeProjectToGeometryOpConfig()
{
	return makeTrajectoryOpConfig(
		RobotInstruction::TrajectoryOpKind::ProjectToGeometry,
		"ops/ProjectToGeometry.json");
}

} // namespace trajectory_algo
