#include "MirrorOp.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind MirrorOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Mirror;
}

const char* MirrorOp::displayName(const bool chinese) const
{
	return chinese ? "镜像" : "Mirror";
}

TrajectoryOpCapability MirrorOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor MirrorOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Mirror;
	op.scope = defaultScope;
	return op;
}

std::vector<TrajectoryOpParamField> MirrorOp::paramFields() const
{
	return {
		messageParamField(
			"hint.notImplemented",
			"Mirror block is not implemented yet.",
			"镜像块尚未实现，预览与应用暂不可用。"),
	};
}

bool MirrorOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	if (errMsg)
	{
		*errMsg = "Mirror block is not implemented yet.";
	}
	return false;
}

std::string MirrorOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return chinese ? "镜像 | 未实现" : "Mirror | N/A";
}

bool MirrorOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> MirrorOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	(void)op;
	return {};
}

} // namespace trajectory_algo
