/// @file RobotCollisionSettings.cpp
/// @brief RobotCollisionSettings JSON 读写

#include "RobotCollisionSettings.h"

namespace RobotCollision
{
void writeSettingsToJson(const Settings& s, nlohmann::json& out)
{
	out = nlohmann::json::object();
	out["enabled"] = s.enabled;
	out["securityMarginMm"] = s.securityMarginMm;
}

bool readSettingsFromJson(const nlohmann::json& in, Settings& out)
{
	if (!in.is_object())
		return false;
	Settings s;
	if (in.contains("enabled") && in["enabled"].is_boolean())
		s.enabled = in["enabled"].get<bool>();
	if (in.contains("securityMarginMm") && in["securityMarginMm"].is_number())
		s.securityMarginMm = in["securityMarginMm"].get<double>();
	if (s.securityMarginMm < 0.0)
		s.securityMarginMm = 0.0;
	out = s;
	return true;
}

} // namespace RobotCollision
