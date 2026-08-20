#ifndef ROBOTSCENE_ROBOTEXTERNALAXES_H
#define ROBOTSCENE_ROBOTEXTERNALAXES_H

/// @file RobotExternalAxes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人实例外部轴：平移/旋转 × 机器人基座/工件；多轴链式合成

#include "robot_scene_global.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace RobotExternal
{
/// 兼容旧工程 JSON；新配置以 motionType + attachment 为准
enum class ROBOT_SCENE_API RobotExternalAxisKind : int
{
	LinearRail = 0,
	Turntable = 1
};

enum class ROBOT_SCENE_API RobotExternalMotionType : int
{
	Translate = 0,
	Rotate = 1
};

enum class ROBOT_SCENE_API RobotExternalAttachment : int
{
	RobotBase = 0,
	Workpiece = 1
};

struct ROBOT_SCENE_API RobotExternalAxisConfig
{
	bool enabled = true;
	std::string displayName = "Rail";
	std::string jointName = "rail_joint";
	RobotExternalAxisKind kind = RobotExternalAxisKind::LinearRail;
	RobotExternalMotionType motionType = RobotExternalMotionType::Translate;
	RobotExternalAttachment attachment = RobotExternalAttachment::RobotBase;
	/// 兼容旧字段；normalize 时与 motionType 同步
	bool isPrismatic = true;
	double lower = 0.0;
	double upper = 1000.0;
	double home = 0.0;
	/// 附着体局部系：平移方向或旋转轴
	double axis[3]{1.0, 0.0, 0.0};
	/// 旋转轴过点（局部 mm）；Translate 可忽略
	double originMm[3]{0.0, 0.0, 0.0};
	/// Workpiece 必填：场景 backendId
	std::string boundBackendId;
	/// 可选工作架；空=backend 根
	std::string workingFrameId;
};

struct ROBOT_SCENE_API RobotExternalAxisConfigSet
{
	std::vector<RobotExternalAxisConfig> axes;
};

inline constexpr const char* kExtContextExternalAxisQMm = "context.externalAxisQMm";
inline constexpr const char* kExtContextExternalAxisQCsv = "context.externalAxisQCsv";
inline constexpr const char* kExtContextExternalAxisDir = "context.externalAxisDir";
/// REP：相对工作架的 TCP（优先于纯 T_p0 重建目标）
inline constexpr const char* kExtContextWorkingTcpTransMmCsv = "context.workingTcpTransMmCsv";
inline constexpr const char* kExtContextWorkingTcpQuatCsv = "context.workingTcpQuatCsv";

ROBOT_SCENE_API bool hasEnabledExternalAxes(const RobotExternalAxisConfigSet& set);
ROBOT_SCENE_API bool hasEnabledWorkpieceExternalAxes(const RobotExternalAxisConfigSet& set);
ROBOT_SCENE_API const RobotExternalAxisConfig* firstEnabledExternalAxis(const RobotExternalAxisConfigSet& set);
ROBOT_SCENE_API const RobotExternalAxisConfig* firstEnabledWorkpieceAxis(const RobotExternalAxisConfigSet& set);
ROBOT_SCENE_API std::vector<const RobotExternalAxisConfig*> enabledExternalAxes(const RobotExternalAxisConfigSet& set);
ROBOT_SCENE_API std::vector<const RobotExternalAxisConfig*>
enabledExternalAxesForAttachment(const RobotExternalAxisConfigSet& set, RobotExternalAttachment attachment);
ROBOT_SCENE_API std::vector<int> enabledExternalAxisIndices(const RobotExternalAxisConfigSet& set);
ROBOT_SCENE_API std::vector<int> enabledExternalAxisIndicesForAttachment(const RobotExternalAxisConfigSet& set,
																		   RobotExternalAttachment attachment);

/// 首个启用 Workpiece 的 boundBackendId；无则空
ROBOT_SCENE_API std::string primaryWorkpieceBackendId(const RobotExternalAxisConfigSet& set);
/// workingFrameId 非空则用之，否则 = boundBackendId
ROBOT_SCENE_API std::string resolveWorkingFrameId(const RobotExternalAxisConfigSet& set);

/// T_work_world = W_eff * offsetW0Local（offset 为空/单位 → 用 W_eff）
ROBOT_SCENE_API void composeWorkpieceWorkingFrameWorld(const double w0ColumnMajor[16],
													   const RobotExternalAxisConfigSet& set,
													   const std::string& boundBackendId,
													   const std::vector<double>& qValues,
													   const double offsetW0Local[16], double outWorkWorld[16]);

/// T_p0_work = inv(P0_world) * T_work_world
ROBOT_SCENE_API bool composeWorkpieceWorkingFrameInRobotP0(const double p0WorldColumnMajor[16],
														   const double w0ColumnMajor[16],
														   const RobotExternalAxisConfigSet& set,
														   const std::string& boundBackendId,
														   const std::vector<double>& qValues,
														   const double offsetW0Local[16], double outTp0Work[16]);

ROBOT_SCENE_API void mat4IdentityColumnMajor(double out[16]);
ROBOT_SCENE_API bool mat4InvertRigidColumnMajor(const double in[16], double out[16]);
ROBOT_SCENE_API void mat4MulColumnMajor16(const double a[16], const double b[16], double out[16]);

/// Workpiece 未绑 backend 时返回 false
ROBOT_SCENE_API bool validateExternalAxisConfig(const RobotExternalAxisConfig& cfg, std::string* errMsg = nullptr);
ROBOT_SCENE_API bool validateExternalAxisConfigSet(const RobotExternalAxisConfigSet& set, std::string* errMsg = nullptr);

ROBOT_SCENE_API void normalizeExternalAxisConfig(RobotExternalAxisConfig& cfg);
ROBOT_SCENE_API RobotExternalAxisConfig makeDefaultLinearRailConfig();
ROBOT_SCENE_API RobotExternalAxisConfig makeDefaultRotateAxisConfig(RobotExternalAttachment attachment);

ROBOT_SCENE_API void writeExternalAxisConfigSetToJson(const RobotExternalAxisConfigSet& set, nlohmann::json& out);
ROBOT_SCENE_API bool readExternalAxisConfigSetFromJson(const nlohmann::json& in, RobotExternalAxisConfigSet& out);

ROBOT_SCENE_API std::string encodeExternalAxisQCsv(const std::vector<double>& qs);
ROBOT_SCENE_API std::vector<double> parseExternalAxisQCsv(const std::string& csv);

/// 仅合成 attachment=RobotBase 的轴：P_eff = P0 * Π T_i(q_i)
ROBOT_SCENE_API void composeBasePlacementWithExternalAxis(const double p0ColumnMajor[16],
														  const RobotExternalAxisConfigSet& set,
														  const std::vector<double>& qValues,
														  double outColumnMajor[16]);
/// 兼容单标量（取首个启用 RobotBase 轴）
ROBOT_SCENE_API void composeBasePlacementWithExternalAxis(const double p0ColumnMajor[16],
														  const RobotExternalAxisConfigSet& set, double qMm,
														  double outColumnMajor[16]);

ROBOT_SCENE_API void unbakeBasePlacementExternalAxis(const double pEffColumnMajor[16],
													 const RobotExternalAxisConfigSet& set,
													 const std::vector<double>& qValues, double outP0ColumnMajor[16]);
ROBOT_SCENE_API void unbakeBasePlacementExternalAxis(const double pEffColumnMajor[16],
													 const RobotExternalAxisConfigSet& set, double qMm,
													 double outP0ColumnMajor[16]);

/// 合成绑定到指定 backend 的 Workpiece 轴：W_eff = W0 * Π T_i(q_i)
ROBOT_SCENE_API void composeWorkpiecePlacementWithExternalAxis(const double w0ColumnMajor[16],
															   const RobotExternalAxisConfigSet& set,
															   const std::string& boundBackendId,
															   const std::vector<double>& qValues,
															   double outColumnMajor[16]);

ROBOT_SCENE_API void unbakeWorkpiecePlacementExternalAxis(const double wEffColumnMajor[16],
														  const RobotExternalAxisConfigSet& set,
														  const std::string& boundBackendId,
														  const std::vector<double>& qValues,
														  double outW0ColumnMajor[16]);

/// 单轴增量变换（局部系）写入 out 4x4 列主序
ROBOT_SCENE_API void makeAxisMotionColumnMajor(const RobotExternalAxisConfig& cfg, double q, double out[16]);

} // namespace RobotExternal

#endif // ROBOTSCENE_ROBOTEXTERNALAXES_H
