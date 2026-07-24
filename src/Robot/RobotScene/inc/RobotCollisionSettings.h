#ifndef ROBOTSCENE_ROBOTCOLLISIONSETTINGS_H
#define ROBOTSCENE_ROBOTCOLLISIONSETTINGS_H

/// @file RobotCollisionSettings.h
/// @brief 文档级碰撞检测开关与安全余量

#include "robot_scene_global.h"

#include <json.hpp>

namespace RobotCollision
{
struct ROBOT_SCENE_API Settings
{
	bool enabled = false;
	double securityMarginMm = 1.0;
};

ROBOT_SCENE_API void writeSettingsToJson(const Settings& s, nlohmann::json& out);
ROBOT_SCENE_API bool readSettingsFromJson(const nlohmann::json& in, Settings& out);

} // namespace RobotCollision

#endif // ROBOTSCENE_ROBOTCOLLISIONSETTINGS_H
