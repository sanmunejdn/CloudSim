#pragma once

#include "RobotInstructionModel.h"
#include "robot_scene_global.h"

#include <json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace RobotInstruction
{

ROBOT_SCENE_API std::shared_ptr<Base> createFromJson(const nlohmann::json& j, std::string* errMsg = nullptr);
ROBOT_SCENE_API nlohmann::json toJson(const Base& ins);
ROBOT_SCENE_API std::vector<std::shared_ptr<Base>> createListFromJson(
	const nlohmann::json& arr,
	std::string* errMsg = nullptr);

} // namespace RobotInstruction
