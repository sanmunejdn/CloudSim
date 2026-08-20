#ifndef ROBOTSCENE_ROBOTCOLLISIONSETTINGS_H
#define ROBOTSCENE_ROBOTCOLLISIONSETTINGS_H

/// @file RobotCollisionSettings.h
/// @brief 文档级碰撞：开关、余量、黑白名单

#include "robot_scene_global.h"

#include <json.hpp>

#include <string>
#include <vector>

namespace RobotCollision
{
struct ROBOT_SCENE_API Settings
{
	bool enabled = false;
	double securityMarginMm = 1.0;
	/// 白名单 backendId：组内互不检；仅与黑名单互检
	std::vector<std::string> whiteListBackendIds;
	/// 黑名单 backendId：组内互不检；仅与白名单互检
	std::vector<std::string> blackListBackendIds;
};

ROBOT_SCENE_API void writeSettingsToJson(const Settings& s, nlohmann::json& out);
ROBOT_SCENE_API bool readSettingsFromJson(const nlohmann::json& in, Settings& out);

} // namespace RobotCollision

#endif // ROBOTSCENE_ROBOTCOLLISIONSETTINGS_H
