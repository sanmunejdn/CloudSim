#pragma once

#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"

#include <string>

namespace trajectory_algo
{

inline bool applyUnifiedPathOp(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	std::string* errMsg)
{
	return RobotInstruction::applyUnifiedTrajectoryOp(op, traj, errMsg);
}

} // namespace trajectory_algo
