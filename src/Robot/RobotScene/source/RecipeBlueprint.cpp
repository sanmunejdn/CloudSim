#include "RecipeBlueprint.h"

#include "ProcessFlowPresetLoader.h"

namespace RobotInstruction
{

std::vector<TrajectoryOpDescriptor> buildRecipePreset(const RecipeKind kind)
{
	const char* presetId = nullptr;
	switch (kind)
	{
	case RecipeKind::Weld:
		presetId = "weld";
		break;
	case RecipeKind::Glue:
		presetId = "glue";
		break;
	case RecipeKind::Grind:
		presetId = "grind";
		break;
	}
	if (presetId)
	{
		return buildRecipePresetFromId(presetId, {}, nullptr);
	}
	return {};
}

} // namespace RobotInstruction
