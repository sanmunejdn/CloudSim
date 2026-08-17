#ifndef ROBOTURDF_URDFNUMERICALIK_H
#define ROBOTURDF_URDFNUMERICALIK_H

/// @file UrdfNumericalIk.h
/// @brief URDF 几何雅可比 + DLS 位姿数值 IK（纯运动学核）

#include "UrdfIkSolverOptions.h"
#include "robot_urdf_global.h"

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

/// 臂位姿 DLS；失败返回空向量
ROBOT_URDF_API std::vector<double> solveArmPoseDampedLeastSquares(const QString& urdfPath, const QString& ikLink,
																  const UrdfPoseIkTarget& target,
																  std::vector<double> seedJointRad,
																  const UrdfIkSolverOptions& options,
																  std::string* failReason = nullptr);

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFNUMERICALIK_H
