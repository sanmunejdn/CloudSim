/// @file UrdfNumericalIk.cpp
/// @brief URDF 臂位姿 DLS 主路径：KinematicCore

#include "UrdfNumericalIk.h"

#include "KinematicCoreUrdfIk.h"

#include <cmath>

namespace UrdfRobotLoader
{
std::vector<double> solveArmPoseDampedLeastSquares(const QString& urdfPath, const QString& ikLink,
												   const UrdfPoseIkTarget& target, std::vector<double> q,
												   const UrdfIkSolverOptions& options, std::string* failReason)
{
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
	return solveArmPoseViaKinematicCore(urdfPath, ikLink, target, std::move(q), options, failReason);
}

} // namespace UrdfRobotLoader
