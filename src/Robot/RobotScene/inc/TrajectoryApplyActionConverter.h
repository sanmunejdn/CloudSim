#pragma once

#include "ProgramEditCommand.h"
#include "RobotInstructionProgram.h"
#include "robot_scene_global.h"

#include <TrajectoryApplyAction.h>

#include <string>
#include <vector>

namespace RobotInstruction
{

ROBOT_SCENE_API std::vector<ProgramEditStack::CommandPtr> convertApplyActionsToCommands(
	const std::vector<trajectory_algo::TrajectoryApplyAction>& actions,
	const RobotProgram& program,
	InstructionProgramDocument& doc,
	std::string* errMsg);

} // namespace RobotInstruction
