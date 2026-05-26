#include "ReorderOp.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind ReorderOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Reorder;
}

const char* ReorderOp::displayName(const bool chinese) const
{
	return chinese ? "重排" : "Reorder";
}

TrajectoryOpCapability ReorderOp::capabilities() const
{
	return TrajectoryOpCapability::None;
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
			"hint.notImplemented",
			"Reorder block is not implemented yet.",
			"重排块尚未实现，预览与应用暂不可用。"),
	};
}

bool ReorderOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	if (errMsg)
	{
		*errMsg = "Reorder block is not implemented yet.";
	}
	return false;
}

std::string ReorderOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return chinese ? "重排 | 未实现" : "Reorder | N/A";
}

bool ReorderOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> ReorderOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	(void)op;
	return {};
}

} // namespace trajectory_algo
