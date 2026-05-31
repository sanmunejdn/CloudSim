#include "DuplicateOp.h"

#include "TrajectoryOpFormat.h"

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
	return TrajectoryOpCapability::ApplyStructuralEdit;
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

bool DuplicateOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> DuplicateOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	if (ctx.program)
	{
		RobotInstruction::RobotProgramCatalog catalog;
		std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(op.scope, *ctx.program);
		if (ids.empty())
		{
			return {};
		}
	}
	TrajectoryApplyAction action{};
	action.kind = TrajectoryApplyActionKind::DuplicateInstruction;
	action.scope = op.scope;
	action.duplicateCount = op.duplicateCount;
	return { action };
}

} // namespace trajectory_algo
