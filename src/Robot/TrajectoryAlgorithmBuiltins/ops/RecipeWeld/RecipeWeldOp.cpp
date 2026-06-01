#include "RecipeWeldOp.h"

#include <cstdio>

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind RecipeWeldOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::RecipeWeld;
}

const char* RecipeWeldOp::displayName(const bool chinese) const
{
	return chinese ? "焊缝配方" : "Weld Recipe";
}

TrajectoryOpCapability RecipeWeldOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor RecipeWeldOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::RecipeWeld;
	op.scope = defaultScope;
	op.recipe.version = 1;
	op.recipe.resampleStepMm = 5.0;
	op.recipe.normalOffsetMm = 0.0;
	op.recipe.speedMmPerSec = 100.0;
	op.recipe.blendRadiusMm = 2.0;
	return op;
}

std::vector<TrajectoryOpParamField> RecipeWeldOp::paramFields() const
{
	return {
		intParamField("recipe.version", "Version", "版本", 1, 999, 1, 0, "recipe"),
		doubleParamField("recipe.resampleStepMm", "Step", "点距", "mm", 0.1, 100.0, 0.1, 5.0, 1, "recipe"),
		doubleParamField("recipe.normalOffsetMm", "Normal Offset", "法向偏移", "mm", -1000.0, 1000.0, 0.1, 0.0, 2, "recipe"),
		doubleParamField("recipe.speedMmPerSec", "Speed", "焊接速度", "mm/s", 1.0, 5000.0, 1.0, 100.0, 3, "recipe"),
		doubleParamField("recipe.blendRadiusMm", "Blend Radius", "圆角", "mm", 0.0, 1000.0, 0.1, 2.0, 4, "recipe"),
	};
}

bool RecipeWeldOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (op.recipe.resampleStepMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "recipe.resampleStepMm must be > 0";
		}
		return false;
	}
	return true;
}

std::string RecipeWeldOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	char buffer[256];
	std::snprintf(
		buffer,
		sizeof(buffer),
		chinese ? "焊缝配方 v%d | 点距%.2f | 偏移%.2f | 圆角%.2f"
				: "Weld Recipe v%d | Step %.2f | Offset %.2f | Blend %.2f",
		op.recipe.version,
		op.recipe.resampleStepMm,
		op.recipe.normalOffsetMm,
		op.recipe.blendRadiusMm);
	return buffer;
}

bool RecipeWeldOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> RecipeWeldOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	(void)op;
	return {};
}

} // namespace trajectory_algo

