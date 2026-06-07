// 管道执行期 scope 解析与活动程序上下文
#pragma once

#include "RobotProgramCatalog.h"
#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"

#include <cstddef>
#include <unordered_set>
#include <vector>

namespace trajectory_algo
{

void setActiveProgramContext(const RobotInstruction::RobotProgram* program);
const RobotInstruction::RobotProgram* activeProgramContext();

std::unordered_set<std::string> resolveScopeInstructionIds(
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::RobotProgram* program);

std::vector<std::size_t> resolveScopedPointIndices(
	const RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program);

} // namespace trajectory_algo
