#include "RecipeGrindOp.h"

#include <cstdio>

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind RecipeGrindOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::RecipeGrind;
}

const char* RecipeGrindOp::displayName(const bool chinese) const
{
	return chinese ? "打磨配方" : "Grind Recipe";
}

TrajectoryOpCapability RecipeGrindOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor RecipeGrindOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::RecipeGrind;
	op.scope = defaultScope;
	op.recipe.version = 1;
	op.recipe.speedMmPerSec = 80.0;
	return op;
}

std::vector<TrajectoryOpParamField> RecipeGrindOp::paramFields() const
{
	return {
		intParamField("recipe.version", "Version", "版本", 1, 999, 1, 0, "recipe"),
		doubleParamField("recipe.speedMmPerSec", "Speed", "打磨速度", "mm/s", 1.0, 5000.0, 1.0, 80.0, 1, "recipe"),
		messageParamField(
			"recipe.grindHint",
			"Default chain: FrameFromPath -> SmoothPose -> ReachabilityFilter -> ExternalAxisSearch -> EmitToProgram",
			"默认链路：FrameFromPath -> SmoothPose -> ReachabilityFilter -> ExternalAxisSearch -> EmitToProgram",
			2),
	};
}

bool RecipeGrindOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (op.recipe.speedMmPerSec <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "recipe.speedMmPerSec must be > 0";
		}
		return false;
	}
	return true;
}

std::string RecipeGrindOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	char buffer[256];
	std::snprintf(
		buffer,
		sizeof(buffer),
		chinese ? "打磨配方 v%d | 速度%.1f | 外轴搜索开"
				: "Grind Recipe v%d | Speed %.1f | External axis on",
		op.recipe.version,
		op.recipe.speedMmPerSec);
	return buffer;
}

bool RecipeGrindOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> RecipeGrindOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	(void)op;
	return {};
}

} // namespace trajectory_algo

