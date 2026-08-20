#ifndef ROBOTSCENE_ROBOTINSTRUCTIONCONDITION_H
#define ROBOTSCENE_ROBOTINSTRUCTIONCONDITION_H

/// @file RobotInstructionCondition.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotInstructionCondition 接口

#include "robot_scene_global.h"

#include <string>

#include <json.hpp>

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
	/// 优先于 ioPort；空则仍用端口号
	std::string signalName;
	std::string compareLeft;
	std::string compareOp; // eq, ne, lt, le, gt, ge
	double compareRight = 0.0;
};

ROBOT_SCENE_API Condition conditionFromJson(const nlohmann::json& j);
ROBOT_SCENE_API nlohmann::json conditionToJson(const Condition& c);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONCONDITION_H
