/// @file RobotExternalAxes.cpp
/// @brief 机器人外部轴配置 JSON 与门禁辅助

#include "RobotExternalAxes.h"

#include <algorithm>
#include <cmath>

namespace RobotExternal
{
namespace
{
void readVec3(const nlohmann::json& j, double out[3])
{
	if (!j.is_array() || j.size() < 3)
	{
		return;
	}
	out[0] = j[0].get<double>();
	out[1] = j[1].get<double>();
	out[2] = j[2].get<double>();
}

void writeVec3(nlohmann::json& j, const double v[3])
{
	j = nlohmann::json::array({v[0], v[1], v[2]});
}

const char* kindToString(const RobotExternalAxisKind kind)
{
	return kind == RobotExternalAxisKind::Turntable ? "Turntable" : "LinearRail";
}

RobotExternalAxisKind kindFromString(const std::string& s)
{
	if (s == "Turntable" || s == "turntable")
	{
		return RobotExternalAxisKind::Turntable;
	}
	return RobotExternalAxisKind::LinearRail;
}
} // namespace

bool hasEnabledExternalAxes(const RobotExternalAxisConfigSet& set)
{
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (a.enabled)
		{
			return true;
		}
	}
	return false;
}

const RobotExternalAxisConfig* firstEnabledExternalAxis(const RobotExternalAxisConfigSet& set)
{
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (a.enabled)
		{
			return &a;
		}
	}
	return nullptr;
}

void normalizeExternalAxisConfig(RobotExternalAxisConfig& cfg)
{
	if (cfg.displayName.empty())
	{
		cfg.displayName = "Rail";
	}
	if (cfg.jointName.empty())
	{
		cfg.jointName = "rail_joint";
	}
	if (cfg.kind == RobotExternalAxisKind::LinearRail)
	{
		cfg.isPrismatic = true;
	}
	if (cfg.upper < cfg.lower)
	{
		std::swap(cfg.lower, cfg.upper);
	}
	cfg.home = std::clamp(cfg.home, cfg.lower, cfg.upper);
	const double len = std::sqrt(cfg.axis[0] * cfg.axis[0] + cfg.axis[1] * cfg.axis[1] + cfg.axis[2] * cfg.axis[2]);
	if (len < 1e-9)
	{
		cfg.axis[0] = 1.0;
		cfg.axis[1] = 0.0;
		cfg.axis[2] = 0.0;
	}
	else
	{
		cfg.axis[0] /= len;
		cfg.axis[1] /= len;
		cfg.axis[2] /= len;
	}
}

RobotExternalAxisConfig makeDefaultLinearRailConfig()
{
	RobotExternalAxisConfig cfg;
	normalizeExternalAxisConfig(cfg);
	return cfg;
}

void writeExternalAxisConfigSetToJson(const RobotExternalAxisConfigSet& set, nlohmann::json& out)
{
	out = nlohmann::json::object();
	nlohmann::json arr = nlohmann::json::array();
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		nlohmann::json item = nlohmann::json::object();
		item["enabled"] = a.enabled;
		item["displayName"] = a.displayName;
		item["jointName"] = a.jointName;
		item["kind"] = kindToString(a.kind);
		item["isPrismatic"] = a.isPrismatic;
		item["lower"] = a.lower;
		item["upper"] = a.upper;
		item["home"] = a.home;
		writeVec3(item["axis"], a.axis);
		arr.push_back(std::move(item));
	}
	out["axes"] = std::move(arr);
}

bool readExternalAxisConfigSetFromJson(const nlohmann::json& in, RobotExternalAxisConfigSet& out)
{
	out = RobotExternalAxisConfigSet{};
	if (!in.is_object())
	{
		return false;
	}
	if (!in.contains("axes") || !in["axes"].is_array())
	{
		return true;
	}
	for (const auto& item : in["axes"])
	{
		if (!item.is_object())
		{
			continue;
		}
		RobotExternalAxisConfig cfg = makeDefaultLinearRailConfig();
		if (item.contains("enabled") && item["enabled"].is_boolean())
		{
			cfg.enabled = item["enabled"].get<bool>();
		}
		if (item.contains("displayName") && item["displayName"].is_string())
		{
			cfg.displayName = item["displayName"].get<std::string>();
		}
		if (item.contains("jointName") && item["jointName"].is_string())
		{
			cfg.jointName = item["jointName"].get<std::string>();
		}
		if (item.contains("kind") && item["kind"].is_string())
		{
			cfg.kind = kindFromString(item["kind"].get<std::string>());
		}
		if (item.contains("isPrismatic") && item["isPrismatic"].is_boolean())
		{
			cfg.isPrismatic = item["isPrismatic"].get<bool>();
		}
		if (item.contains("lower") && item["lower"].is_number())
		{
			cfg.lower = item["lower"].get<double>();
		}
		if (item.contains("upper") && item["upper"].is_number())
		{
			cfg.upper = item["upper"].get<double>();
		}
		if (item.contains("home") && item["home"].is_number())
		{
			cfg.home = item["home"].get<double>();
		}
		if (item.contains("axis"))
		{
			readVec3(item["axis"], cfg.axis);
		}
		normalizeExternalAxisConfig(cfg);
		out.axes.push_back(std::move(cfg));
	}
	return true;
}

void mat4MulColumnMajor(const double a[16], const double b[16], double out[16])
{
	double tmp[16];
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			tmp[c * 4 + r] = a[0 * 4 + r] * b[c * 4 + 0] + a[1 * 4 + r] * b[c * 4 + 1] + a[2 * 4 + r] * b[c * 4 + 2] +
							 a[3 * 4 + r] * b[c * 4 + 3];
		}
	}
	for (int i = 0; i < 16; ++i)
	{
		out[i] = tmp[i];
	}
}

void makeTranslateColumnMajor(const double tx, const double ty, const double tz, double out[16])
{
	for (int i = 0; i < 16; ++i)
	{
		out[i] = 0.0;
	}
	out[0] = out[5] = out[10] = out[15] = 1.0;
	// 与 BackendMat4/OSG 同序：平移在 (3,0)/(3,1)/(3,2)，不是 Eigen 的 [12..14]
	out[3] = tx;
	out[7] = ty;
	out[11] = tz;
}

void composeBasePlacementWithExternalAxis(const double p0ColumnMajor[16], const RobotExternalAxisConfigSet& set,
										  const double qMm, double outColumnMajor[16])
{
	for (int i = 0; i < 16; ++i)
	{
		outColumnMajor[i] = p0ColumnMajor[i];
	}
	const RobotExternalAxisConfig* rail = firstEnabledExternalAxis(set);
	if (!rail || !rail->isPrismatic)
	{
		return;
	}
	double t[16];
	makeTranslateColumnMajor(rail->axis[0] * qMm, rail->axis[1] * qMm, rail->axis[2] * qMm, t);
	mat4MulColumnMajor(p0ColumnMajor, t, outColumnMajor);
}

void unbakeBasePlacementExternalAxis(const double pEffColumnMajor[16], const RobotExternalAxisConfigSet& set,
									 const double qMm, double outP0ColumnMajor[16])
{
	for (int i = 0; i < 16; ++i)
	{
		outP0ColumnMajor[i] = pEffColumnMajor[i];
	}
	const RobotExternalAxisConfig* rail = firstEnabledExternalAxis(set);
	if (!rail || !rail->isPrismatic)
	{
		return;
	}
	double t[16];
	makeTranslateColumnMajor(rail->axis[0] * qMm, rail->axis[1] * qMm, rail->axis[2] * qMm, t);
	// T^{-1} = Trans(-t)
	double tInv[16];
	makeTranslateColumnMajor(-rail->axis[0] * qMm, -rail->axis[1] * qMm, -rail->axis[2] * qMm, tInv);
	mat4MulColumnMajor(pEffColumnMajor, tInv, outP0ColumnMajor);
}

} // namespace RobotExternal
