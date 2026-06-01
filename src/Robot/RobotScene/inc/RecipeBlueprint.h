#pragma once

#include "RawTrajectory.h"
#include "TrajectoryPipelineTypes.h"
#include "robot_scene_global.h"

#include <string>
#include <vector>

namespace RobotInstruction
{

enum class ROBOT_SCENE_API RecipeKind
{
	Weld = 0,
	Glue,
	Grind
};

ROBOT_SCENE_API bool isRecipeOpKind(TrajectoryOpKind kind);
ROBOT_SCENE_API bool recipeKindFromOpKind(TrajectoryOpKind kind, RecipeKind& out);

ROBOT_SCENE_API std::vector<TrajectoryOpDescriptor> buildRecipePreset(RecipeKind kind);

ROBOT_SCENE_API bool applyRecipeDescriptorToRawTrajectory(
	const TrajectoryOpDescriptor& op,
	RawTrajectory& raw,
	std::string* errMsg = nullptr);

} // namespace RobotInstruction

