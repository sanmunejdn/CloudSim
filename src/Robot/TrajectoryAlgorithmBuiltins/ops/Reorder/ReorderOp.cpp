// Reorder 原子块：重排 scope 内路点顺序
#include "ReorderOp.h"

#include "TrajectoryOpPathApply.h"

#include <cstdio>

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
	return op;
}

std::vector<TrajectoryOpParamField> ReorderOp::paramFields() const
{
	return {
		messageParamField(
			"reorder.fixedPoseHint",
			"Lock all orientations to the first waypoint orientation in current scope.",
			"将作用域内所有姿态固定为首个轨迹点姿态。"),
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
	return chinese ? "固定姿态 | 以首点为基准" : "Fixed Orientation | Use first waypoint";
}

bool ReorderOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	std::string* errMsg) const
{
	return applyUnifiedPathOp(op, traj, errMsg);
}

} // namespace trajectory_algo
