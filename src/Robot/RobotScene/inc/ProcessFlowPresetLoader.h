#ifndef ROBOTSCENE_PROCESSFLOWPRESETLOADER_H
#define ROBOTSCENE_PROCESSFLOWPRESETLOADER_H

/// @file ProcessFlowPresetLoader.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief ProcessFlowPresetLoader 接口

#include "robot_scene_global.h"

#include "RecipeBlueprint.h"
#include "TrajectoryPipelineTypes.h"

#include <string>
#include <vector>

namespace RobotInstruction
{
struct ROBOT_SCENE_API ProcessFlowPresetEntry
{
	std::string id;
	std::string labelZh;
	std::string labelEn;
	std::vector<TrajectoryOpKind> ops;
	std::vector<TrajectoryOpDescriptor> pipeline;
};

ROBOT_SCENE_API std::vector<ProcessFlowPresetEntry> loadProcessFlowPresets(const std::string& resourceBaseDir = {},
																		   std::string* errMsg = nullptr);

ROBOT_SCENE_API std::vector<TrajectoryOpDescriptor> buildRecipePresetFromId(const std::string& presetId,
																			const std::string& resourceBaseDir = {},
																			std::string* errMsg = nullptr);

ROBOT_SCENE_API TrajectoryOpKind trajectoryOpKindFromPresetToken(const std::string& token, bool* ok = nullptr);

} // namespace RobotInstruction

#endif // ROBOTSCENE_PROCESSFLOWPRESETLOADER_H
