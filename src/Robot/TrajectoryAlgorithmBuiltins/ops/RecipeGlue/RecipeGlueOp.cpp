#include "RecipeGlueOp.h"

#include <cstdio>

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind RecipeGlueOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::RecipeGlue;
}

const char* RecipeGlueOp::displayName(const bool chinese) const
{
	return chinese ? "涂胶配方" : "Glue Recipe";
}

TrajectoryOpCapability RecipeGlueOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor RecipeGlueOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::RecipeGlue;
	op.scope = defaultScope;
	op.recipe.version = 1;
	op.recipe.resampleStepMm = 2.0;
	op.recipe.normalOffsetMm = 5.0;
	op.recipe.lateralOffsetMm = 3.0;
	op.recipe.speedMmPerSec = 50.0;
	return op;
}

std::vector<TrajectoryOpParamField> RecipeGlueOp::paramFields() const
{
	return {
		intParamField("recipe.version", "Version", "版本", 1, 999, 1, 0, "recipe"),
		doubleParamField("recipe.resampleStepMm", "Step", "点距", "mm", 0.1, 100.0, 0.1, 2.0, 1, "recipe"),
		doubleParamField("recipe.normalOffsetMm", "Normal Lift", "法向抬高", "mm", -1000.0, 1000.0, 0.1, 5.0, 2, "recipe"),
		doubleParamField("recipe.lateralOffsetMm", "Lateral Offset", "侧向偏移", "mm", -1000.0, 1000.0, 0.1, 3.0, 3, "recipe"),
		doubleParamField("recipe.speedMmPerSec", "Speed", "涂胶速度", "mm/s", 1.0, 5000.0, 1.0, 50.0, 4, "recipe"),
	};
}

bool RecipeGlueOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
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

std::string RecipeGlueOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	char buffer[256];
	std::snprintf(
		buffer,
		sizeof(buffer),
		chinese ? "涂胶配方 v%d | 点距%.2f | 抬高%.2f | 侧偏%.2f | 速度%.1f"
				: "Glue Recipe v%d | Step %.2f | Lift %.2f | Lateral %.2f | Speed %.1f",
		op.recipe.version,
		op.recipe.resampleStepMm,
		op.recipe.normalOffsetMm,
		op.recipe.lateralOffsetMm,
		op.recipe.speedMmPerSec);
	return buffer;
}

bool RecipeGlueOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> RecipeGlueOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	(void)op;
	return {};
}

} // namespace trajectory_algo

