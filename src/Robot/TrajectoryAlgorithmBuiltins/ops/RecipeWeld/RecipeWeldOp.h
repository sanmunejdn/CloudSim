#pragma once

#include "ITrajectoryOp.h"

namespace trajectory_algo
{

class RecipeWeldOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* displayName(bool chinese) const override;
	TrajectoryOpCapability capabilities() const override;
	RobotInstruction::TrajectoryOpDescriptor makeDefaultDescriptor(
		const RobotInstruction::OpScope& defaultScope) const override;
	std::vector<TrajectoryOpParamField> paramFields() const override;
	bool validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const override;
	std::string formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, bool chinese) const override;
	bool contributePreviewTransform(
		const RobotInstruction::TrajectoryOpDescriptor& op,
		const std::vector<std::string>& targetIds,
		PreviewTransformStep& out) const override;
	std::vector<TrajectoryApplyAction> buildApplyActions(
		const TrajectoryOpContext& ctx,
		const RobotInstruction::TrajectoryOpDescriptor& op) const override;
};

} // namespace trajectory_algo

