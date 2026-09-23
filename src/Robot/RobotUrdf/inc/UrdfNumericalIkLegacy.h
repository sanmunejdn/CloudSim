#ifndef ROBOTURDF_URDFNUMERICALIKLEGACY_H
#define ROBOTURDF_URDFNUMERICALIKLEGACY_H

/// @file UrdfNumericalIkLegacy.h
/// @brief legacy BFS 雅可比 DLS，仅 SelfTest / 对照

#include "robot_urdf_global.h"

#include "UrdfIkSolverOptions.h"

#include <QString>
#include <string>
#include <vector>

namespace UrdfRobotLoader
{
struct UrdfPoseIkTarget;

ROBOT_URDF_API std::vector<double> solveArmPoseViaUrdfJacobianLegacy(const QString& urdfPath, const QString& ikLink,
																	 const UrdfPoseIkTarget& target,
																	 std::vector<double> q,
																	 const UrdfIkSolverOptions& options,
																	 std::string* failReason = nullptr);
} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFNUMERICALIKLEGACY_H
