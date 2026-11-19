/// @file UrdfNumericalIk.cpp
/// @brief URDF 臂位姿 IK 入口（走 Registry）

#include "UrdfNumericalIk.h"

#include "PoseIkSolverRegistry.h"

namespace UrdfRobotLoader
{
std::vector<double> solveArmPoseDampedLeastSquares(const QString& urdfPath, const QString& ikLink,
												   const UrdfPoseIkTarget& target, std::vector<double> q,
												   const UrdfIkSolverOptions& options, std::string* failReason,
												   IkConvergenceStatus* status)
{
	return solveArmPoseViaRegistry(urdfPath, ikLink, target, std::move(q), options, failReason, status);
}

} // namespace UrdfRobotLoader
