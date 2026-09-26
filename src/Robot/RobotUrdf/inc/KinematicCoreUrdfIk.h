#ifndef ROBOTURDF_KINEMATICCOREURDFIK_H
#define ROBOTURDF_KINEMATICCOREURDFIK_H

/// @file KinematicCoreUrdfIk.h
/// @brief KinematicCore 图 IK（位置）；失败返回空 vector

#include "robot_urdf_global.h"

#include "UrdfNumericalIk.h"

#include <QString>
#include <QVector>
#include <string>
#include <vector>

namespace UrdfRobotLoader
{
/// KinematicCore 图 IK；Soft 仅当 options.allowApproximateOrientation
ROBOT_URDF_API std::vector<double> solveArmPoseViaKinematicCore(const QString& urdfPath, const QString& ikLink,
																const UrdfPoseIkTarget& target, std::vector<double> q,
																const UrdfIkSolverOptions& options,
																std::string* failReason = nullptr,
																IkConvergenceStatus* status = nullptr);

/// Core FK + 几何雅可比（与 computeLinkPoseAndGeometricJacobian 同 task 布局）
ROBOT_URDF_API bool computeLinkPoseAndJacobianViaCore(const QString& urdfPath, const QVector<double>& jointAnglesRad,
													  const QString& linkName, double outPosMm[3], double* outQuatXyzw,
													  std::vector<double>& outJ_rowMajor, bool includeOrientation,
													  double orientationWeight = 300.0,
													  QString* errorMessage = nullptr);
} // namespace UrdfRobotLoader

#endif // ROBOTURDF_KINEMATICCOREURDFIK_H
