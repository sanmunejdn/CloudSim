// Reorder 原子块：按 scope 统一姿态
#include "ReorderOp.h"

#include "UnifiedTrajectorySemanticMath.h"

#include <cstdio>
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind ReorderOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Reorder;
}

const char* ReorderOp::displayName(const bool chinese) const
{
	return chinese ? "固定姿态" : "Fixed Orientation";
}

TrajectoryOpCapability ReorderOp::capabilities() const
{
	return TrajectoryOpCapability::PreviewPoseTransform;
}

RobotInstruction::TrajectoryOpDescriptor ReorderOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Reorder;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> ReorderOp::paramFields() const
{
	return {
		messageParamField(
			"reorder.fixedPoseHint",
			"Lock all orientations to the first waypoint orientation in current scope.",
			"将当前作用域内所有路点姿态锁定为第一个路点的姿态。"),
	};
}

bool ReorderOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ReorderOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return chinese ? "固定姿态 | 首点姿态" : "Fixed Orientation | Use first waypoint";
}

bool ReorderOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	const TrajectoryOpExecutionContext& ctx,
	std::string* errMsg) const
{
	return applyReorderInScope(op, traj, ctx.program);
}

} // namespace trajectory_algo
