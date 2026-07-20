#ifndef ROBOTSCENE_RECIPEBLUEPRINT_H
#define ROBOTSCENE_RECIPEBLUEPRINT_H

/// @file RecipeBlueprint.h
/// @brief RecipeBlueprint 接口

#include "robot_scene_global.h"

#include "TrajectoryPipelineTypes.h"

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

#endif // ROBOTSCENE_RECIPEBLUEPRINT_H
