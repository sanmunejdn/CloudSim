#ifndef ROBOTSCENE_ROBOTTEACHIK_H
#define ROBOTSCENE_ROBOTTEACHIK_H

/// @file RobotTeachIk.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 示教 IK：T_base_target + 可选外轴（1～N DOF）→ URDF 数值 IK

#include "robot_scene_global.h"

#include <QString>
#include <string>
#include <vector>

#include <BackendDataBase.h>
#include <RigidTransform.h>
#include <UrdfIkSolverOptions.h>

namespace RobotTeachIk
{
struct ROBOT_SCENE_API TeachIkExternalAxis
{
	bool enabled = false;
	bool isPrismatic = true;
	double axis[3]{1.0, 0.0, 0.0};
	/// Rotate：轴过点（臂基座局部 mm）；Translate 可忽略
	double originMm[3]{0.0, 0.0, 0.0};
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

/// 多轴示教：仅启用轴列表；qExternal[i] 对齐 axes[i]
/// configIndex 指向完整配置下标，便于结果展开为 config 对齐向量
struct ROBOT_SCENE_API TeachIkExternalAxisSlot
{
	int configIndex = -1;
	bool isPrismatic = true;
	double axis[3]{1.0, 0.0, 0.0};
	double originMm[3]{0.0, 0.0, 0.0};
	double lower = 0.0;
	double upper = 1000.0;
};

struct ROBOT_SCENE_API TeachIkExternalAxisDof
{
	std::vector<TeachIkExternalAxisSlot> axes;
	std::vector<double> qExternal;
	bool optimizeExternal = false;
	double externalDeltaPriorWeight = 0.0;
	bool adaptiveExternalDamping = true;

	bool active() const { return !axes.empty() && qExternal.size() == axes.size(); }
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
	/// 求解器容差/阻尼（与 maxIkIterations 合并进 options.maxIterations）
	UrdfRobotLoader::UrdfIkSolverOptions options{};
	/// 兼容单轴；externalAxes 非空时优先
	TeachIkExternalAxis externalAxis{};
	TeachIkExternalAxisDof externalAxes{};
	/// 完整配置轴数；>0 时结果 externalAxisQs 按 config 下标对齐（未用槽填 0）
	int externalAxisConfigCount = 0;
	/// 非空时 IK/FK 经 KinematicModelRegistry（robot:<sceneRootId>）
	std::string registryKey;
};

struct ROBOT_SCENE_API TeachIkResult
{
	bool ok = false;
	std::vector<double> jointRad;
	double residualTcpMm = 0.0;
	/// 首轴（兼容）：优先首个 RobotBase/启用槽
	double externalAxisQ = 0.0;
	/// 优先 config 下标对齐（externalAxisConfigCount>0）；否则与启用列表同长
	std::vector<double> externalAxisQs;
	std::string error;
};

/// 示教 IK：T_base_target 与 T_flange_tool → 法兰目标 → URDF 数值 IK
ROBOT_SCENE_API TeachIkResult solveTeachIk(const TeachIkContext& ctx);

/// 拖动联立：臂固定 / 投影种子联立 / 自适应联立 多候选代价选优（多轴时走向量路径）
ROBOT_SCENE_API TeachIkResult solveTeachIkCoordinatedDrag(const TeachIkContext& ctx, double qExternalHintMm,
														  bool hasExternalHint);

/// 固定外轴时，将基座系目标平移到「外轴=0」臂 IK 目标（单平移轴）
ROBOT_SCENE_API void applyExternalAxisToTargetPos(const double axis[3], double qExt, double posInOut[3]);

/// 多轴 unbake：逆序消外轴；Rotate 绕 origin 逆旋（可改姿态）
ROBOT_SCENE_API void applyExternalAxesToTarget(const TeachIkExternalAxisDof& dof, double posInOut[3],
											   double quatXyzwInOut[4]);

} // namespace RobotTeachIk

#endif // ROBOTSCENE_ROBOTTEACHIK_H
