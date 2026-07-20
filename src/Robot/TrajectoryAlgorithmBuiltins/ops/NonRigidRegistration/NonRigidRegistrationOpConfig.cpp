/// @file NonRigidRegistrationOpConfig.cpp
/// @brief NonRigidRegistrationOpConfig 实现

#include "NonRigidRegistrationOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeNonRigidRegistrationOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::NonRigidRegistration,
								  "ops/NonRigidRegistration.json");
}

} // namespace trajectory_algo
