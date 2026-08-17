#ifndef ROBOTURDF_URDFIKSOLVEROPTIONS_H
#define ROBOTURDF_URDFIKSOLVEROPTIONS_H

/// @file UrdfIkSolverOptions.h
/// @brief 数值位姿 IK 容差/阻尼/迭代（与历史 TeachIk 魔法数对齐）

#include "robot_urdf_global.h"

namespace UrdfRobotLoader
{
struct ROBOT_URDF_API UrdfIkSolverOptions
{
	double lambda = 1e-2;
	double positionToleranceMm = 1e-2;
	double orientationToleranceRad = 0.1 * 3.14159265358979323846 / 180.0;
	double orientationWeight = 300.0;
	/// 0：沿用内部默认步进 0.2 rad
	double maxJointStepRad = 0.0;
	/// 0：默认 180
	int maxIterations = 0;
};

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFIKSOLVEROPTIONS_H
