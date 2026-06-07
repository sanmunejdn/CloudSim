// Duplicate 原子块：复制 scope 内路点
#include "DuplicateOp.h"

#include "TrajectoryOpFormat.h"
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

RobotInstruction::TrajectoryOpDescriptor DuplicateOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Duplicate;
	op.scope = defaultScope;
	op.duplicateCount = 1;
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
	if (op.duplicateCount < 1)
	{
		if (errMsg)
		{
			*errMsg = "duplicate count must be >= 1";
		}
		return false;
	}
	return true;
}

std::string DuplicateOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	return std::string(displayName(chinese)) + " | " + scopeKindLabel(op.scope.kind, chinese)
		+ (chinese ? " | 份数=" : " | Count=") + std::to_string(op.duplicateCount);
}

bool DuplicateOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	std::string* errMsg) const
{
	(void)errMsg;
	const std::vector<std::size_t> indices =
		resolveScopedPointIndices(traj, op.scope, activeProgramContext());
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
	for (int copy = 0; copy < op.duplicateCount; ++copy)
	{
		traj.points.insert(
			traj.points.begin() + static_cast<std::ptrdiff_t>(insertPos + static_cast<std::size_t>(copy) * chunk.size()),
			chunk.begin(),
			chunk.end());
	}
	return true;
}

} // namespace trajectory_algo
