/// @file ToWorkpieceInHandOpConfig.cpp
/// @brief ToWorkpieceInHand 算子配置

#include "ToWorkpieceInHandOpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeToWorkpieceInHandOpConfig()
{
	return makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::ToWorkpieceInHand, "ops/ToWorkpieceInHand.json");
}

} // namespace trajectory_algo
