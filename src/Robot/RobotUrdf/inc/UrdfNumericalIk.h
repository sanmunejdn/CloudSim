#ifndef ROBOTURDF_URDFNUMERICALIK_H
#define ROBOTURDF_URDFNUMERICALIK_H

/// @file UrdfNumericalIk.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief URDF 位姿 IK 入口（解析优先，否则 KinematicCore DLS）

#include "robot_urdf_global.h"

#include "UrdfIkSolverOptions.h"

#include <QString>
#include <string>
#include <vector>

namespace UrdfRobotLoader
{
struct ROBOT_URDF_API UrdfPoseIkTarget
{
	double posMm[3]{0.0, 0.0, 0.0};
	bool hasOrientation = false;
	/// xyzw
	double quatXyzw[4]{0.0, 0.0, 0.0, 1.0};
};

/// 臂位姿 IK：球形腕解析（若适用）→ KinematicCore DLS；status 可选
ROBOT_URDF_API std::vector<double> solveArmPoseDampedLeastSquares(const QString& urdfPath, const QString& ikLink,
																  const UrdfPoseIkTarget& target,
																  std::vector<double> seedJointRad,
																  const UrdfIkSolverOptions& options,
																  std::string* failReason = nullptr,
																  IkConvergenceStatus* status = nullptr);

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFNUMERICALIK_H
