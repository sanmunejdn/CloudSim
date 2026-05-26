#pragma once

#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <json.hpp>
#include <string>
#include <vector>

namespace trajectory_algo
{

class ITrajectoryOp;

TRAJECTORY_ALGORITHM_API nlohmann::json toJson(const RobotInstruction::TrajectoryOpDescriptor& op);
TRAJECTORY_ALGORITHM_API bool fromJson(
	const nlohmann::json& j,
	RobotInstruction::TrajectoryOpDescriptor& out,
	std::string* errMsg = nullptr);

TRAJECTORY_ALGORITHM_API nlohmann::json pipelineToJson(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops);
TRAJECTORY_ALGORITHM_API bool pipelineFromJson(
	const nlohmann::json& j,
	std::vector<RobotInstruction::TrajectoryOpDescriptor>& out,
	std::string* errMsg = nullptr);

} // namespace trajectory_algo
