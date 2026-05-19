#pragma once

#include "robot_scene_global.h"

#include <BackendDataBase.h>
#include <RigidTransform.h>

#include <QString>
#include <string>
#include <vector>

namespace RobotTeachIk
{
struct ROBOT_SCENE_API TeachIkContext
{
	QString urdfPath;
	/// URDF link used for numerical IK Jacobian (typically flange / \c context.flangeLinkName).
	QString ikLinkName;
	/// Tool origin in robot base frame (\c T_base_target).
	engine::RigidTransform T_base_target;
	std::vector<double> seedJointRad;
	bool useOrientation = true;
	BackendMat4 T_flange_tool = BackendMat4::identity();
};

struct ROBOT_SCENE_API TeachIkResult
{
	bool ok = false;
	std::vector<double> jointRad;
	double residualTcpMm = 0.0;
	std::string error;
};

/// Interactive teach IK: flange target from \c T_base_target and \c T_flange_tool, URDF numerical IK.
ROBOT_SCENE_API TeachIkResult solveTeachIk(const TeachIkContext& ctx);

} // namespace RobotTeachIk
