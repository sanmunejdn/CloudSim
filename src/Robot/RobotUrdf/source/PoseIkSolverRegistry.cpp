/// @file PoseIkSolverRegistry.cpp
/// @brief 解析 / DLS 注册调度

#include "PoseIkSolverRegistry.h"

#include "IkSeedExpand.h"
#include "KinematicCoreDlsIk.h"
#include "SphericalWristAnalyticalIk.h"
#include "UrdfIkMachineProfile.h"

#include <cmath>

namespace UrdfRobotLoader
{
namespace
{
SphericalWristAnalyticalIk g_sphericalWristSolver;
KinematicCoreDlsIk g_dlsSolver;

bool allowAnalyticalForPath(const QString& urdfPath, const UrdfIkMachineProfile& mach)
{
	if (mach.analyticalFamily == AnalyticalIkFamily::None)
	{
		return false;
	}
	if (mach.analyticalFamily == AnalyticalIkFamily::SphericalWrist6R ||
		mach.forceSphericalWrist == SphericalWristForce::Yes)
	{
		return true;
	}
	if (mach.forceSphericalWrist == SphericalWristForce::No)
	{
		return false;
	}
	return g_sphericalWristSolver.isEligible(urdfPath);
}
} // namespace

std::vector<PoseIkCandidate> collectPoseIkCandidates(const QString& urdfPath, const QString& ikLink,
													 const UrdfPoseIkTarget& target, std::vector<double> seedJointRad,
													 const UrdfIkSolverOptions& options, const bool includeAnalytical,
													 const bool includeNumerical, std::string* failReason)
{
	std::vector<PoseIkCandidate> out;
	std::string lastFail;
	if (includeAnalytical)
	{
		const UrdfIkMachineProfile mach = resolveIkMachineProfile(urdfPath);
		if (allowAnalyticalForPath(urdfPath, mach))
		{
			UrdfIkSolverOptions anaOpt = options;
			anaOpt.raceAnalyticalAndNumerical = false;
			std::string anaFail;
			auto anaCands = SphericalWristAnalyticalIk::solveAll(urdfPath, ikLink, target, seedJointRad, anaOpt,
																 &anaFail);
			for (auto& ac : anaCands)
			{
				PoseIkCandidate c;
				c.q = std::move(ac.q);
				c.branchId = ac.wristBranch; // 流水线再 encode 肘符号
				c.status = IkConvergenceStatus::HardConverged;
				c.solverName = g_sphericalWristSolver.name();
				out.push_back(std::move(c));
			}
			if (out.empty() && !anaFail.empty())
			{
				lastFail = std::move(anaFail);
			}
		}
	}
	if (includeNumerical)
	{
		std::string numFail;
		IkConvergenceStatus st = IkConvergenceStatus::Failed;
		std::vector<double> q = g_dlsSolver.solve(urdfPath, ikLink, target, seedJointRad, options, &numFail, &st);
		if (!q.empty())
		{
			PoseIkCandidate c;
			c.q = std::move(q);
			c.status = st;
			c.solverName = g_dlsSolver.name();
			out.push_back(std::move(c));
		}
		else if (!numFail.empty())
		{
			lastFail = std::move(numFail);
		}
	}
	if (out.empty() && failReason)
	{
		*failReason = lastFail;
	}
	return out;
}

std::vector<double> solveArmPoseViaRegistry(const QString& urdfPath, const QString& ikLink,
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
	{ return g_dlsSolver.solve(urdfPath, ikLink, target, q, options, fr, st); };

	if (options.preferNumericalOnly)
	{
		return runNumerical(failReason, status);
	}

	const UrdfIkMachineProfile mach = resolveIkMachineProfile(urdfPath);
	if (!allowAnalyticalForPath(urdfPath, mach))
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

	return runNumerical(failReason, status);
}

} // namespace UrdfRobotLoader
