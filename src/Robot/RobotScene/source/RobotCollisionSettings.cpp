/// @file RobotCollisionSettings.cpp
/// @brief RobotCollisionSettings JSON 读写

#include "RobotCollisionSettings.h"

namespace RobotCollision
{
namespace
{
void writeIdArray(nlohmann::json& out, const char* key, const std::vector<std::string>& ids)
{
	nlohmann::json arr = nlohmann::json::array();
	for (const std::string& id : ids)
	{
		if (!id.empty())
			arr.push_back(id);
	}
	out[key] = std::move(arr);
}

void readIdArray(const nlohmann::json& in, const char* key, std::vector<std::string>& out)
{
	out.clear();
	if (!in.contains(key) || !in[key].is_array())
		return;
	for (const auto& el : in[key])
	{
		if (!el.is_string())
			continue;
		const std::string id = el.get<std::string>();
		if (!id.empty())
			out.push_back(id);
	}
}
} // namespace

void writeSettingsToJson(const Settings& s, nlohmann::json& out)
{
	out = nlohmann::json::object();
	out["enabled"] = s.enabled;
	out["securityMarginMm"] = s.securityMarginMm;
	writeIdArray(out, "whiteListBackendIds", s.whiteListBackendIds);
	writeIdArray(out, "blackListBackendIds", s.blackListBackendIds);
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
	readIdArray(in, "whiteListBackendIds", s.whiteListBackendIds);
	readIdArray(in, "blackListBackendIds", s.blackListBackendIds);
	out = s;
	return true;
}

} // namespace RobotCollision
