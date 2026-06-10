// OffsetAlongNormal 原子块：沿法向偏移路径点
#include "OffsetAlongNormalOp.h"

#include "TrajectoryOpFormat.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind OffsetAlongNormalOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::OffsetAlongNormal;
}

const char* OffsetAlongNormalOp::displayName(const bool chinese) const
{
	return chinese ? "法向偏移" : "OffsetAlongNormal";
}

TrajectoryOpCapability OffsetAlongNormalOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor OffsetAlongNormalOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::OffsetAlongNormal;
	op.scope = defaultScope;
	return op;
}

std::vector<TrajectoryOpParamField> OffsetAlongNormalOp::paramFields() const
{
	return {
		doubleParamField("offset.offsetMm", "Offset", "法向偏移", "mm", -1e5, 1e5, 0.01, 0.0, 0),
	};
}

bool OffsetAlongNormalOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string OffsetAlongNormalOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool OffsetAlongNormalOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	const TrajectoryOpExecutionContext& ctx,
	std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	offsetAlongNormalUnifiedInScope(
		traj,
		op.scope,
		ctx.program,
		op.pathOffset.offsetMm);
	return true;
}

} // namespace trajectory_algo
