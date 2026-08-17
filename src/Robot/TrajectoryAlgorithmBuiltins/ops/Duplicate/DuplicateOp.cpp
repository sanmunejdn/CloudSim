/// @file DuplicateOp.cpp
/// @brief Duplicate 轨迹算子

// Duplicate 原子块：复制 scope 内路点
#include "DuplicateOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "TrajectoryUnifiedScope.h"

#include <string>

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind DuplicateOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Duplicate;
}

const char* DuplicateOp::displayName(const bool chinese) const
{
	return chinese ? "复制" : "Duplicate";
}

TrajectoryOpCapability DuplicateOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
DuplicateOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Duplicate;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	writeDuplicateCount(op.params, 1);

	return op;
}

std::vector<TrajectoryOpParamField> DuplicateOp::paramFields() const
{
	return {
		intParamField("structural.duplicateCount", "Count", "份数", 1, 99, 1, 0, "structural"),
	};
}

bool DuplicateOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (parseDuplicateCount(op.params) < 1)
	{
		if (errMsg)
		{
			*errMsg = "duplicate count must be >= 1";
		}
		return false;
	}
	return true;
}

std::string DuplicateOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	return std::string(displayName(chinese)) + " | " + scopeKindLabel(op.scope.kind, chinese) +
		   (chinese ? " | 份数=" : " | Count=") + std::to_string(parseDuplicateCount(op.params));
}

bool DuplicateOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
							  RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
							  std::string* errMsg) const
{
	(void)errMsg;
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, op.scope, ctx.program);
	if (indices.empty())
	{
		return true;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> chunk;
	chunk.reserve(indices.size());
	for (const std::size_t idx : indices)
	{
		chunk.push_back(traj.points[idx]);
	}
	const std::size_t insertPos = indices.back() + 1U;
	for (int copy = 0; copy < parseDuplicateCount(op.params); ++copy)
	{
		traj.points.insert(traj.points.begin() +
							   static_cast<std::ptrdiff_t>(insertPos + static_cast<std::size_t>(copy) * chunk.size()),
						   chunk.begin(), chunk.end());
	}
	return true;
}

} // namespace trajectory_algo
