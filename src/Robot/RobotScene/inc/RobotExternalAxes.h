#ifndef ROBOTSCENE_ROBOTEXTERNALAXES_H
#define ROBOTSCENE_ROBOTEXTERNALAXES_H

/// @file RobotExternalAxes.h
/// @brief 机器人实例外部轴配置（地轨/变位机）；未启用时不参与联动求解

#include "robot_scene_global.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace RobotExternal
{
enum class ROBOT_SCENE_API RobotExternalAxisKind : int
{
	LinearRail = 0,
	Turntable = 1
};

struct ROBOT_SCENE_API RobotExternalAxisConfig
{
	bool enabled = true;
	std::string displayName = "Rail";
	std::string jointName = "rail_joint";
	RobotExternalAxisKind kind = RobotExternalAxisKind::LinearRail;
	bool isPrismatic = true;
	double lower = 0.0;
	double upper = 1000.0;
	double home = 0.0;
	/// 基座系单位方向（地轨平移轴）
	double axis[3]{1.0, 0.0, 0.0};
};

struct ROBOT_SCENE_API RobotExternalAxisConfigSet
{
	std::vector<RobotExternalAxisConfig> axes;
};

/// 规划结果写回指令扩展键
inline constexpr const char* kExtContextExternalAxisQMm = "context.externalAxisQMm";
inline constexpr const char* kExtContextExternalAxisDir = "context.externalAxisDir";

ROBOT_SCENE_API bool hasEnabledExternalAxes(const RobotExternalAxisConfigSet& set);
ROBOT_SCENE_API const RobotExternalAxisConfig* firstEnabledExternalAxis(const RobotExternalAxisConfigSet& set);

ROBOT_SCENE_API void normalizeExternalAxisConfig(RobotExternalAxisConfig& cfg);
ROBOT_SCENE_API RobotExternalAxisConfig makeDefaultLinearRailConfig();

ROBOT_SCENE_API void writeExternalAxisConfigSetToJson(const RobotExternalAxisConfigSet& set, nlohmann::json& out);
ROBOT_SCENE_API bool readExternalAxisConfigSetFromJson(const nlohmann::json& in, RobotExternalAxisConfigSet& out);

/// P_eff = P0 * Trans(q·axis)；Mat4 与 BackendMat4/OSG 同序（平移在 index 3/7/11）
ROBOT_SCENE_API void composeBasePlacementWithExternalAxis(const double p0ColumnMajor[16],
														  const RobotExternalAxisConfigSet& set, double qMm,
														  double outColumnMajor[16]);
/// P0 = P_eff * inv(Trans(q·axis))
ROBOT_SCENE_API void unbakeBasePlacementExternalAxis(const double pEffColumnMajor[16],
													 const RobotExternalAxisConfigSet& set, double qMm,
													 double outP0ColumnMajor[16]);

} // namespace RobotExternal

#endif // ROBOTSCENE_ROBOTEXTERNALAXES_H
