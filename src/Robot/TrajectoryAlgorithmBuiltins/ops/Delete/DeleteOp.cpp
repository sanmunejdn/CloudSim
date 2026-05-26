#include "DeleteOp.h"

#include "TrajectoryOpFormat.h"

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
	return TrajectoryOpCapability::ApplyStructuralEdit;
}

RobotInstruction::TrajectoryOpDescriptor DeleteOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Delete;
	op.scope = defaultScope;
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

std::string DeleteOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	return std::string(displayName(chinese)) + " | " + scopeKindLabel(op.scope.kind, chinese);
}

bool DeleteOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> DeleteOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	TrajectoryApplyAction action{};
	action.kind = TrajectoryApplyActionKind::RemoveInstruction;
	action.scope = op.scope;
	return { action };
}

} // namespace trajectory_algo
