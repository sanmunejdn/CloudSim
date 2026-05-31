#include "TrajectoryApplyActionConverter.h"

#include "RobotProgramCatalog.h"

#include <algorithm>
#include <unordered_map>

namespace RobotInstruction
{
namespace
{
std::string scopeCacheKey(const OpScope& scope, const bool expandGroup)
{
	std::string key = std::to_string(static_cast<int>(scope.kind));
	key.push_back('|');
	key += scope.groupId;
	key.push_back('|');
	key += std::to_string(scope.pointFrom);
	key.push_back('|');
	key += std::to_string(scope.pointTo);
	key.push_back('|');
	key += expandGroup ? "1" : "0";
	if (scope.kind == OpScope::Kind::InstructionIds)
	{
		for (const std::string& id : scope.instructionIds)
		{
			key.push_back('|');
			key += id;
		}
	}
	return key;
}

const std::vector<std::string>& resolveScopeIdsCached(
	const OpScope& scope,
	const RobotProgram& program,
	RobotProgramCatalog& catalog,
	std::unordered_map<std::string, std::vector<std::string>>& scopeCache,
	const bool expandGroup)
{
	const std::string key = scopeCacheKey(scope, expandGroup);
	auto it = scopeCache.find(key);
	if (it != scopeCache.end())
	{
		return it->second;
	}
	std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(scope, program);
	if (expandGroup && scope.kind == OpScope::Kind::Group)
	{
		ids = catalog.expandToMotionWaypointIds(program, ids);
	}
	return scopeCache.emplace(key, std::move(ids)).first->second;
}
} // namespace

std::vector<ProgramEditStack::CommandPtr> convertApplyActionsToCommands(
	const std::vector<trajectory_algo::TrajectoryApplyAction>& actions,
	const RobotProgram& program,
	InstructionProgramDocument& doc,
	std::string* errMsg)
{
	(void)doc;
	std::vector<ProgramEditStack::CommandPtr> out;
	RobotProgramCatalog catalog;
	std::unordered_map<std::string, std::vector<std::string>> scopeCache;
	for (const trajectory_algo::TrajectoryApplyAction& action : actions)
	{
		switch (action.kind)
		{
		case trajectory_algo::TrajectoryApplyActionKind::TransformSegment:
		{
			for (const TrajectoryOpDescriptor& op : action.transformOps)
			{
				const bool expandGroup = (op.scope.kind == OpScope::Kind::Group);
				const std::vector<std::string>& ids =
					resolveScopeIdsCached(op.scope, program, catalog, scopeCache, expandGroup);
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
			const std::vector<std::string>& ids =
				resolveScopeIdsCached(action.scope, program, catalog, scopeCache, false);
			std::vector<ProgramEditStack::CommandPtr> batch;
			batch.reserve(ids.size());
			for (const std::string& id : ids)
			{
				batch.push_back(std::make_shared<RemoveInstructionCommand>(id));
			}
			if (!batch.empty())
			{
				out.push_back(std::make_shared<CompositeProgramEditCommand>(std::move(batch)));
			}
			break;
		}
		case trajectory_algo::TrajectoryApplyActionKind::DuplicateInstruction:
		{
			const std::vector<std::string>& ids =
				resolveScopeIdsCached(action.scope, program, catalog, scopeCache, false);
			std::vector<ProgramEditStack::CommandPtr> batch;
			batch.reserve(ids.size() * static_cast<size_t>(std::max(1, action.duplicateCount)));
			for (const std::string& id : ids)
			{
				const int duplicateCount = std::max(1, action.duplicateCount);
				for (int i = 0; i < duplicateCount; ++i)
				{
					batch.push_back(std::make_shared<DuplicateInstructionCommand>(id, 0));
				}
			}
			if (!batch.empty())
			{
				out.push_back(std::make_shared<CompositeProgramEditCommand>(std::move(batch)));
			}
			break;
		}
		}
	}
	if (out.empty() && errMsg)
	{
		*errMsg = "no effective operations";
	}
	return out;
}

} // namespace RobotInstruction
