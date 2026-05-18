#pragma once

#include "robot_scene_global.h"

#include <json.hpp>

#include <string>

namespace RobotInstruction
{

enum class ROBOT_SCENE_API ConditionKind
{
	Always = 0,
	Never,
	Io,
	Compare
};

struct ROBOT_SCENE_API Condition
{
	ConditionKind kind = ConditionKind::Always;
	int ioPort = 0;
	bool ioEquals = false;
	std::string compareLeft;
	std::string compareOp; // eq, ne, lt, le, gt, ge
	double compareRight = 0.0;
};

ROBOT_SCENE_API Condition conditionFromJson(const nlohmann::json& j);
ROBOT_SCENE_API nlohmann::json conditionToJson(const Condition& c);

} // namespace RobotInstruction
