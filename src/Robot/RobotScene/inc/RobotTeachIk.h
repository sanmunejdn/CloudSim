#ifndef ROBOTSCENE_ROBOTTEACHIK_H
#define ROBOTSCENE_ROBOTTEACHIK_H

/// @file RobotTeachIk.h
/// @brief 示教 IK：T_base_target 与 T_flange_tool → 法兰目标 → URDF 数值 IK

#include "robot_scene_global.h"

#include <QString>
#include <string>
#include <vector>

#include <BackendDataBase.h>
#include <RigidTransform.h>

namespace RobotTeachIk
{
struct ROBOT_SCENE_API TeachIkContext
{
	QString urdfPath;
	/// 数值 IK 雅可比连杆（通常法兰 / context.flangeLinkName）
	QString ikLinkName;
	/// 基座下工具原点 T_base_target
	engine::RigidTransform T_base_target;
	std::vector<double> seedJointRad;
	bool useOrientation = true;
	BackendMat4 T_flange_tool = BackendMat4::identity();
	/// 0=全迭代默认180；拖动示教宜 8–12，小步收敛防跳解
	int maxIkIterations = 0;
};

struct ROBOT_SCENE_API TeachIkResult
{
	bool ok = false;
	std::vector<double> jointRad;
	double residualTcpMm = 0.0;
	std::string error;
};

/// 示教 IK：T_base_target 与 T_flange_tool → 法兰目标 → URDF 数值 IK
ROBOT_SCENE_API TeachIkResult solveTeachIk(const TeachIkContext& ctx);

} // namespace RobotTeachIk

#endif // ROBOTSCENE_ROBOTTEACHIK_H
