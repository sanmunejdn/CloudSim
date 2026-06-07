// TrajectoryUnifiedScope 实现
#include "TrajectoryUnifiedScope.h"

#include <algorithm>

namespace trajectory_algo
{
namespace
{
const RobotInstruction::RobotProgram* g_activeProgramContext = nullptr;
} // namespace

void setActiveProgramContext(const RobotInstruction::RobotProgram* program)
{
	g_activeProgramContext = program;
}

const RobotInstruction::RobotProgram* activeProgramContext()
{
	return g_activeProgramContext;
}

std::unordered_set<std::string> resolveScopeInstructionIds(
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::RobotProgram* program)
{
	std::unordered_set<std::string> ids;
	if (program)
	{
		RobotInstruction::RobotProgramCatalog catalog;
		for (const std::string& id : catalog.resolveOpScopeInstructionIds(scope, *program))
		{
			ids.insert(id);
		}
		if (!ids.empty())
		{
			return ids;
		}
	}
	if (scope.kind == RobotInstruction::OpScope::Kind::InstructionIds)
	{
		for (const std::string& id : scope.instructionIds)
		{
			ids.insert(id);
		}
	}
	else if (scope.kind == RobotInstruction::OpScope::Kind::PointIndexRange)
	{
		const int from = std::max(1, scope.pointFrom);
		const int to = std::max(from, scope.pointTo);
		for (std::size_t i = 0; i < traj.points.size(); ++i)
		{
			const int oneBased = static_cast<int>(i) + 1;
			if (oneBased >= from && oneBased <= to && !traj.points[i].sourceInstructionId.empty())
			{
				ids.insert(traj.points[i].sourceInstructionId);
			}
		}
	}
	else if (scope.kind == RobotInstruction::OpScope::Kind::EntireProgram)
	{
		for (const RobotInstruction::UnifiedTrajectoryPoint& point : traj.points)
		{
			if (!point.sourceInstructionId.empty())
			{
				ids.insert(point.sourceInstructionId);
			}
		}
	}
	return ids;
}

std::vector<std::size_t> resolveScopedPointIndices(
	const RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program)
{
	std::vector<std::size_t> indices;
	const std::unordered_set<std::string> ids = resolveScopeInstructionIds(scope, traj, program);
	if (!ids.empty())
	{
		for (std::size_t i = 0; i < traj.points.size(); ++i)
		{
			if (ids.count(traj.points[i].sourceInstructionId) != 0)
			{
				indices.push_back(i);
			}
		}
		return indices;
	}
	if (scope.kind == RobotInstruction::OpScope::Kind::PointIndexRange)
	{
		const int from = std::max(1, scope.pointFrom);
		const int to = std::max(from, scope.pointTo);
		for (std::size_t i = 0; i < traj.points.size(); ++i)
		{
			const int oneBased = static_cast<int>(i) + 1;
			if (oneBased >= from && oneBased <= to)
			{
				indices.push_back(i);
			}
		}
		return indices;
	}
	if (scope.kind == RobotInstruction::OpScope::Kind::EntireProgram)
	{
		indices.reserve(traj.points.size());
		for (std::size_t i = 0; i < traj.points.size(); ++i)
		{
			indices.push_back(i);
		}
	}
	return indices;
}

} // namespace trajectory_algo
