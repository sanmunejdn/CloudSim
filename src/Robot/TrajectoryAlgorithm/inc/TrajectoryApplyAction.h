#pragma once

#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <string>
#include <vector>

namespace trajectory_algo
{

enum class TrajectoryApplyActionKind
{
	TransformSegment = 0,
	RemoveInstruction,
	DuplicateInstruction
};

struct TRAJECTORY_ALGORITHM_API TrajectoryApplyAction
{
	TrajectoryApplyActionKind kind = TrajectoryApplyActionKind::TransformSegment;
	RobotInstruction::OpScope scope{};
	std::vector<std::string> instructionIds;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> transformOps;
	int duplicateCount = 1;
};

} // namespace trajectory_algo
