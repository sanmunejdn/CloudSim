/// @file UrdfNumericalIk.cpp
/// @brief URDF 臂位姿 IK：球形腕解析优先，否则 KinematicCore DLS

#include "UrdfNumericalIk.h"

#include "IkSeedExpand.h"
#include "KinematicCoreUrdfIk.h"
#include "SphericalWristAnalyticalIk.h"
#include "UrdfIkMachineProfile.h"

#include <cmath>

namespace UrdfRobotLoader
{
namespace
{
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

	const std::vector<double>& seedRef =
		(!options.seedAttractionQ.empty() && options.seedAttractionQ.size() == q.size()) ? options.seedAttractionQ : q;

	auto runNumerical = [&](std::string* fr, IkConvergenceStatus* st) -> std::vector<double>
	{
		UrdfIkSolverOptions numOpt = options;
		numOpt.preferNumericalOnly = true;
		numOpt.raceAnalyticalAndNumerical = false;
		return solveArmPoseViaKinematicCore(urdfPath, ikLink, target, q, numOpt, fr, st);
	};

	if (options.preferNumericalOnly)
	{
		return runNumerical(failReason, status);
	}

	const UrdfIkMachineProfile mach = resolveIkMachineProfile(urdfPath);
	const bool allowAnalytical =
		mach.analyticalFamily != AnalyticalIkFamily::None &&
		(mach.analyticalFamily == AnalyticalIkFamily::SphericalWrist6R || g_sphericalWristSolver.isEligible(urdfPath));
	if (!allowAnalytical)
	{
		return runNumerical(failReason, status);
	}

	if (options.raceAnalyticalAndNumerical)
	{
		std::string analyticalFail;
		std::string numericalFail;
		IkConvergenceStatus analyticalStatus = IkConvergenceStatus::Failed;
		IkConvergenceStatus numericalStatus = IkConvergenceStatus::Failed;
		UrdfIkSolverOptions anaOpt = options;
		anaOpt.raceAnalyticalAndNumerical = false;
		std::vector<double> qAnalytical =
			g_sphericalWristSolver.solve(urdfPath, ikLink, target, q, anaOpt, &analyticalFail, &analyticalStatus);
		std::vector<double> qNumerical = runNumerical(&numericalFail, &numericalStatus);
		if (!qAnalytical.empty() && !qNumerical.empty())
		{
			const double dA = wrappedJointDistance(qAnalytical, seedRef);
			const double dN = wrappedJointDistance(qNumerical, seedRef);
			if (dN + 1e-9 < dA)
			{
				if (status)
				{
					*status = numericalStatus;
				}
				if (failReason)
				{
					failReason->clear();
				}
				return qNumerical;
			}
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
		if (!qNumerical.empty())
		{
			if (status)
			{
				*status = numericalStatus;
			}
			if (failReason)
			{
				failReason->clear();
			}
			return qNumerical;
		}
		if (failReason)
		{
			// 优先数值失败原因，避免 "not spherical-wrist" 掩盖真因
			*failReason = numericalFail.empty() ? analyticalFail : numericalFail;
		}
		return {};
	}

	{
		std::string analyticalFail;
		IkConvergenceStatus analyticalStatus = IkConvergenceStatus::Failed;
		UrdfIkSolverOptions anaOpt = options;
		anaOpt.raceAnalyticalAndNumerical = false;
		std::vector<double> qAnalytical =
			g_sphericalWristSolver.solve(urdfPath, ikLink, target, q, anaOpt, &analyticalFail, &analyticalStatus);
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
	}

	return solveArmPoseViaKinematicCore(urdfPath, ikLink, target, std::move(q), options, failReason, status);
}

} // namespace UrdfRobotLoader
