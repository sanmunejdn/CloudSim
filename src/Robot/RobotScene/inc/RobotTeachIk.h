#ifndef ROBOTSCENE_ROBOTTEACHIK_H
#define ROBOTSCENE_ROBOTTEACHIK_H

/// @file RobotTeachIk.h
/// @brief 示教 IK：T_base_target + 可选地轨外轴 → URDF 数值 IK

#include "robot_scene_global.h"

#include <QString>
#include <string>
#include <vector>

#include <BackendDataBase.h>
#include <RigidTransform.h>

namespace RobotTeachIk
{
struct ROBOT_SCENE_API TeachIkExternalAxis
{
	bool enabled = false;
	bool isPrismatic = true;
	double axis[3]{1.0, 0.0, 0.0};
	double lower = 0.0;
	double upper = 1000.0;
	/// 入参：seed；出参：最优外轴值（mm 或 rad）
	double qExternal = 0.0;
	/// true：把外轴并入 DLS；false：固定 qExternal 仅解臂
	bool optimizeExternal = false;
	/// 软约束：抑制无意义的外轴跳动（相对本步 seed）
	double externalDeltaPriorWeight = 0.0;
	/// 按误差沿轨比例自适应外轴阻尼
	bool adaptiveExternalDamping = true;
};

struct ROBOT_SCENE_API TeachIkContext
{
	QString urdfPath;
	/// 数值 IK 雅可比连杆（通常法兰 / context.flangeLinkName）
	QString ikLinkName;
	/// 基座下工具原点 T_base_target（外轴=0 时的基座系）
	engine::RigidTransform T_base_target;
	std::vector<double> seedJointRad;
	bool useOrientation = true;
	BackendMat4 T_flange_tool = BackendMat4::identity();
	/// 0=全迭代默认180；拖动示教宜 8–12，小步收敛防跳解
	int maxIkIterations = 0;
	TeachIkExternalAxis externalAxis{};
};

struct ROBOT_SCENE_API TeachIkResult
{
	bool ok = false;
	std::vector<double> jointRad;
	double residualTcpMm = 0.0;
	double externalAxisQ = 0.0;
	std::string error;
};

/// 示教 IK：T_base_target 与 T_flange_tool → 法兰目标 → URDF 数值 IK
ROBOT_SCENE_API TeachIkResult solveTeachIk(const TeachIkContext& ctx);

/// 拖动联立：臂固定 / 投影种子联立 / 自适应联立 多候选代价选优
ROBOT_SCENE_API TeachIkResult solveTeachIkCoordinatedDrag(const TeachIkContext& ctx, double qExternalHintMm,
														  bool hasExternalHint);

/// 固定外轴时，将基座系目标平移到「外轴=0」臂 IK 目标
ROBOT_SCENE_API void applyExternalAxisToTargetPos(const double axis[3], double qExt, double posInOut[3]);

} // namespace RobotTeachIk

#endif // ROBOTSCENE_ROBOTTEACHIK_H
