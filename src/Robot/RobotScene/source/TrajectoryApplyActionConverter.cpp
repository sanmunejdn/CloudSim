#include "TrajectoryApplyActionConverter.h"

#include "RobotProgramCatalog.h"

namespace RobotInstruction
{

std::vector<ProgramEditStack::CommandPtr> convertApplyActionsToCommands(
	const std::vector<trajectory_algo::TrajectoryApplyAction>& actions,
	const RobotProgram& program,
	InstructionProgramDocument& doc,
	std::string* errMsg)
{
	(void)doc;
	std::vector<ProgramEditStack::CommandPtr> out;
	RobotProgramCatalog catalog;
	for (const trajectory_algo::TrajectoryApplyAction& action : actions)
	{
		switch (action.kind)
		{
		case trajectory_algo::TrajectoryApplyActionKind::TransformSegment:
		{
			for (const TrajectoryOpDescriptor& op : action.transformOps)
			{
				std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(op.scope, program);
				if (op.scope.kind == OpScope::Kind::Group)
				{
					ids = catalog.expandToMotionWaypointIds(program, ids);
				}
				if (!ids.empty())
				{
					out.push_back(std::make_shared<TransformMotionSegmentCommand>(
						ids,
						std::vector<TrajectoryOpDescriptor>{ op }));
				}
			}
			break;
		}
		case trajectory_algo::TrajectoryApplyActionKind::RemoveInstruction:
		{
			const std::vector<std::string> ids =
				catalog.resolveOpScopeInstructionIds(action.scope, program);
			for (const std::string& id : ids)
			{
				out.push_back(std::make_shared<RemoveInstructionCommand>(id));
			}
			break;
		}
		case trajectory_algo::TrajectoryApplyActionKind::DuplicateInstruction:
		{
			const std::vector<std::string> ids =
				catalog.resolveOpScopeInstructionIds(action.scope, program);
			for (const std::string& id : ids)
			{
				out.push_back(std::make_shared<DuplicateInstructionCommand>(id, 0));
			}
			break;
		}
		}
	}
	if (out.empty() && errMsg)
	{
		*errMsg = "no applicable operations";
	}
	return out;
}

} // namespace RobotInstruction
