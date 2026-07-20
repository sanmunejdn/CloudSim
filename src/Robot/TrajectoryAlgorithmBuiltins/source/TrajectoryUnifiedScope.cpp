/// @file TrajectoryUnifiedScope.cpp
/// @brief TrajectoryUnifiedScope 实现

// TrajectoryUnifiedScope 实现
#include "TrajectoryUnifiedScope.h"

#include <algorithm>

namespace trajectory_algo
{
std::unordered_set<std::string> resolveScopeInstructionIds(const RobotInstruction::OpScope& scope,
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

std::vector<std::size_t> resolveScopedPointIndices(const RobotInstruction::UnifiedTrajectory& traj,
												   const RobotInstruction::OpScope& scope,
												   const RobotInstruction::RobotProgram* program)
{
	std::vector<std::size_t> indices;

	// EntireProgram：直接返回所有点索引，跳过指令 ID 匹配
	if (scope.kind == RobotInstruction::OpScope::Kind::EntireProgram)
	{
		indices.reserve(traj.points.size());
		for (std::size_t i = 0; i < traj.points.size(); ++i)
		{
			indices.push_back(i);
		}
		return indices;
	}

	// PointIndexRange：按索引范围筛选
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

	// Group / InstructionIds：按指令 ID 匹配
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
		// ID 匹配命中则返回
		if (!indices.empty())
		{
			return indices;
		}
	}

	// 兜底：轨迹点无 sourceInstructionId（如原始轨迹/特征离散生成）时，
	// 指令 ID 匹配必然失败，此时将作用域视为全量以避免算法静默跳过
	indices.reserve(traj.points.size());
	for (std::size_t i = 0; i < traj.points.size(); ++i)
	{
		indices.push_back(i);
	}
	return indices;
}

} // namespace trajectory_algo
