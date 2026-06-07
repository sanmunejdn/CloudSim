#pragma once

#include "TrajectoryPipelineTypes.h"
#include "robot_scene_global.h"

#include <vector>

namespace RobotInstruction
{

enum class ROBOT_SCENE_API RecipeKind
{
	Weld = 0,
	Glue,
	Grind
};

ROBOT_SCENE_API std::vector<TrajectoryOpDescriptor> buildRecipePreset(RecipeKind kind);

} // namespace RobotInstruction
