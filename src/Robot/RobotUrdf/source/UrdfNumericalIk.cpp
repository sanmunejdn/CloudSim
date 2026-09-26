/// @file UrdfNumericalIk.cpp
/// @brief URDF 臂位姿 IK：球形腕解析优先，否则 KinematicCore DLS

#include "UrdfNumericalIk.h"

#include "KinematicCoreUrdfIk.h"
#include "SphericalWristAnalyticalIk.h"

#include <cmath>

namespace UrdfRobotLoader
{
namespace
{
// 运行时可换成 PinocchioPoseIkSolver；本波不链 SDK
SphericalWristAnalyticalIk g_sphericalWristSolver;
} // namespace

std::vector<double> solveArmPoseDampedLeastSquares(const QString& urdfPath, const QString& ikLink,
												   const UrdfPoseIkTarget& target, std::vector<double> q,
												   const UrdfIkSolverOptions& options, std::string* failReason,
												   IkConvergenceStatus* status)
{
	if (status)
	{
		*status = IkConvergenceStatus::Failed;
	}
	if (urdfPath.isEmpty() || ikLink.isEmpty() || q.empty())
	{
		if (failReason)
		{
			*failReason = "无URDF上下文";
		}
		return {};
	}
	const double targetNormMm = std::sqrt(target.posMm[0] * target.posMm[0] + target.posMm[1] * target.posMm[1] +
										  target.posMm[2] * target.posMm[2]);
	if (targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	std::string analyticalFail;
	IkConvergenceStatus analyticalStatus = IkConvergenceStatus::Failed;
	std::vector<double> qAnalytical =
		g_sphericalWristSolver.solve(urdfPath, ikLink, target, q, options, &analyticalFail, &analyticalStatus);
	if (!qAnalytical.empty())
	{
		if (status)
		{
			*status = analyticalStatus;
		}
		if (failReason)
		{
			failReason->clear();
		}
		return qAnalytical;
	}

	return solveArmPoseViaKinematicCore(urdfPath, ikLink, target, std::move(q), options, failReason, status);
}

} // namespace UrdfRobotLoader
