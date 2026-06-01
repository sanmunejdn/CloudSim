#include "RecipeBlueprint.h"

#include <algorithm>

namespace RobotInstruction
{
namespace
{
TrajectoryOpDescriptor makeOp(const TrajectoryOpKind kind)
{
	TrajectoryOpDescriptor op{};
	op.kind = kind;
	op.scope.kind = OpScope::Kind::Group;
	return op;
}

void stripBuiltInApproachRetract(std::vector<RawTrajectoryOpDescriptor>& ops)
{
	ops.erase(
		std::remove_if(
			ops.begin(),
			ops.end(),
			[](const RawTrajectoryOpDescriptor& op) { return op.kind == RawTrajectoryOpKind::InsertApproachRetract; }),
		ops.end());
}
} // namespace

bool isRecipeOpKind(const TrajectoryOpKind kind)
{
	return kind == TrajectoryOpKind::RecipeWeld
		|| kind == TrajectoryOpKind::RecipeGlue
		|| kind == TrajectoryOpKind::RecipeGrind;
}

bool recipeKindFromOpKind(const TrajectoryOpKind kind, RecipeKind& out)
{
	switch (kind)
	{
	case TrajectoryOpKind::RecipeWeld:
		out = RecipeKind::Weld;
		return true;
	case TrajectoryOpKind::RecipeGlue:
		out = RecipeKind::Glue;
		return true;
	case TrajectoryOpKind::RecipeGrind:
		out = RecipeKind::Grind;
		return true;
	default:
		return false;
	}
}

std::vector<TrajectoryOpDescriptor> buildRecipePreset(const RecipeKind kind)
{
	std::vector<TrajectoryOpDescriptor> out;
	switch (kind)
	{
	case RecipeKind::Weld:
		out.push_back(makeOp(TrajectoryOpKind::RecipeWeld));
		out.push_back(makeOp(TrajectoryOpKind::Approach));
		out.push_back(makeOp(TrajectoryOpKind::Retract));
		break;
	case RecipeKind::Glue:
		out.push_back(makeOp(TrajectoryOpKind::RecipeGlue));
		break;
	case RecipeKind::Grind:
		out.push_back(makeOp(TrajectoryOpKind::RecipeGrind));
		out.push_back(makeOp(TrajectoryOpKind::Approach));
		out.push_back(makeOp(TrajectoryOpKind::Retract));
		break;
	}
	return out;
}

bool applyRecipeDescriptorToRawTrajectory(
	const TrajectoryOpDescriptor& op,
	RawTrajectory& raw,
	std::string* errMsg)
{
	std::vector<RawTrajectoryOpDescriptor> recipeOps;
	RecipeKind recipeKind = RecipeKind::Weld;
	if (!recipeKindFromOpKind(op.kind, recipeKind))
	{
		if (errMsg)
		{
			*errMsg = "not a recipe op";
		}
		return false;
	}
	switch (recipeKind)
	{
	case RecipeKind::Weld:
		recipeOps = rawTrajectoryRecipeWeldDefault();
		stripBuiltInApproachRetract(recipeOps);
		break;
	case RecipeKind::Glue:
		recipeOps = rawTrajectoryRecipeGlueDefault();
		break;
	case RecipeKind::Grind:
		recipeOps = rawTrajectoryRecipeGrindDefault();
		stripBuiltInApproachRetract(recipeOps);
		break;
	}
	for (RawTrajectoryOpDescriptor& rawOp : recipeOps)
	{
		if (rawOp.kind == RawTrajectoryOpKind::Resample && op.recipe.resampleStepMm > 0.0)
		{
			rawOp.stepMm = op.recipe.resampleStepMm;
		}
		if (rawOp.kind == RawTrajectoryOpKind::OffsetAlongNormal)
		{
			rawOp.offsetMm = op.recipe.normalOffsetMm;
		}
		if (rawOp.kind == RawTrajectoryOpKind::OffsetLateral)
		{
			rawOp.lateralMm = op.recipe.lateralOffsetMm;
		}
		if (rawOp.kind == RawTrajectoryOpKind::AssignBlend && op.recipe.blendRadiusMm > 0.0)
		{
			rawOp.blendRadiusMm = op.recipe.blendRadiusMm;
		}
		if (rawOp.kind == RawTrajectoryOpKind::AssignSpeedZone && op.recipe.speedMmPerSec > 0.0)
		{
			rawOp.speedMmPerSec = op.recipe.speedMmPerSec;
		}
	}
	return applyRawTrajectoryPipeline(recipeOps, raw, errMsg);
}

} // namespace RobotInstruction

