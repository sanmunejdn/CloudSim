/// @file DeleteOp.cpp
/// @brief Delete 轨迹算子

// Delete 原子块：删除 scope 内路点
#include "DeleteOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "TrajectoryUnifiedScope.h"

#include <algorithm>

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind DeleteOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Delete;
}

const char* DeleteOp::displayName(const bool chinese) const
{
	return chinese ? "删除" : "Delete";
}

TrajectoryOpCapability DeleteOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
DeleteOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Delete;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> DeleteOp::paramFields() const
{
	return {};
}

bool DeleteOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string DeleteOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	return std::string(displayName(chinese)) + " | " + scopeKindLabel(op.scope.kind, chinese);
}

bool DeleteOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, op.scope, ctx.program);
	if (indices.empty())
	{
		return true;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> kept;
	kept.reserve(traj.points.size());
	for (std::size_t i = 0; i < traj.points.size(); ++i)
	{
		if (std::find(indices.begin(), indices.end(), i) == indices.end())
		{
			kept.push_back(traj.points[i]);
		}
	}
	traj.points = std::move(kept);
	return true;
}

} // namespace trajectory_algo
