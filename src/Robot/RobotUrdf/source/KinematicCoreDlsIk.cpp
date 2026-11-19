/// @file KinematicCoreDlsIk.cpp
/// @brief DLS 求解器适配

#include "KinematicCoreDlsIk.h"

#include "KinematicCoreUrdfIk.h"

namespace UrdfRobotLoader
{
std::vector<double> KinematicCoreDlsIk::solve(const QString& urdfPath, const QString& ikLink,
											  const UrdfPoseIkTarget& target, std::vector<double> seedJointRad,
											  const UrdfIkSolverOptions& options, std::string* failReason,
											  IkConvergenceStatus* status)
{
	UrdfIkSolverOptions opt = options;
	opt.preferNumericalOnly = true;
	opt.raceAnalyticalAndNumerical = false;
	return solveArmPoseViaKinematicCore(urdfPath, ikLink, target, std::move(seedJointRad), opt, failReason, status);
}

} // namespace UrdfRobotLoader
