#include "ReorderOp.h"

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
	return TrajectoryOpCapability::PreviewPoseTransform | TrajectoryOpCapability::ApplyPoseTransform;
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

bool ReorderOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	if (targetIds.empty())
	{
		return false;
	}
	out.kind = PreviewTransformStep::Kind::FixedOrientationToFirst;
	out.referenceId = targetIds.front();
	out.targetIds.clear();
	for (const std::string& id : targetIds)
	{
		out.targetIds.insert(id);
	}
	return !out.targetIds.empty();
}

std::vector<TrajectoryApplyAction> ReorderOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	TrajectoryApplyAction action{};
	action.kind = TrajectoryApplyActionKind::TransformSegment;
	action.transformOps = { op };
	return { action };
}

} // namespace trajectory_algo
