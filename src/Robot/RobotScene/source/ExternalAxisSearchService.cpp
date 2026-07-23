/// @file ExternalAxisSearchService.cpp
/// @brief 外部轴网格搜索实现

#include "ExternalAxisSearchService.h"

#include "RobotTeachIk.h"
#include "UnifiedTrajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace RobotInstruction
{
void ExternalAxisSearchService::setRobotContext(const QString& urdfPath, const QString& ikLinkName,
												const std::vector<double>& seedJointRad)
{
	m_urdfPath = urdfPath;
	m_ikLinkName = ikLinkName;
	m_seedJointRad = seedJointRad;
}

bool ExternalAxisSearchService::search(UnifiedTrajectory& traj,
									   const std::vector<trajectory_algo::ExternalAxisSearchConfigDto>& configs,
									   const bool allowCoupledRefine, std::string* errMsg) const
{
	const trajectory_algo::ExternalAxisSearchConfigDto* cfg = nullptr;
	for (const auto& c : configs)
	{
		if (c.enabled)
		{
			cfg = &c;
			break;
		}
	}
	if (!cfg)
	{
		return true;
	}
	if (m_urdfPath.isEmpty() || m_ikLinkName.isEmpty() || m_seedJointRad.empty())
	{
		if (errMsg)
		{
			*errMsg = "external axis search requires URDF/IK context";
		}
		return false;
	}

	const double span = std::max(0.0, cfg->upper - cfg->lower);
	const int gridN = span < 1e-9 ? 1 : std::clamp(static_cast<int>(span / 50.0) + 1, 5, 41);
	double prevQe = cfg->home;
	std::vector<double> seed = m_seedJointRad;

	traj.ctx.externalAxes.clear();
	ExternalAxisSnapshot snap;
	snap.jointName = cfg->jointName.empty() ? "rail_joint" : cfg->jointName;
	snap.isPrismatic = cfg->isPrismatic;
	snap.positionMmOrRad = cfg->home;

	for (UnifiedTrajectoryPoint& tp : traj.points)
	{
		double bestQe = cfg->home;
		double bestResidual = std::numeric_limits<double>::infinity();
		bool bestOk = false;
		std::vector<double> bestQ = seed;

		for (int i = 0; i < gridN; ++i)
		{
			const double t = gridN <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(gridN - 1);
			const double qe = cfg->lower + t * (cfg->upper - cfg->lower);

			RobotTeachIk::TeachIkContext ctx;
			ctx.urdfPath = m_urdfPath;
			ctx.ikLinkName = m_ikLinkName;
			ctx.seedJointRad = seed;
			ctx.useOrientation = false;
			ctx.maxIkIterations = 80;
			ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(tp.poseMm.x, tp.poseMm.y, tp.poseMm.z,
																				 tp.eulerDeg.x, tp.eulerDeg.y, tp.eulerDeg.z);
			ctx.externalAxis.enabled = true;
			ctx.externalAxis.isPrismatic = cfg->isPrismatic;
			ctx.externalAxis.axis[0] = cfg->axis[0];
			ctx.externalAxis.axis[1] = cfg->axis[1];
			ctx.externalAxis.axis[2] = cfg->axis[2];
			ctx.externalAxis.lower = cfg->lower;
			ctx.externalAxis.upper = cfg->upper;
			ctx.externalAxis.qExternal = qe;
			ctx.externalAxis.optimizeExternal = false;

			const RobotTeachIk::TeachIkResult r = RobotTeachIk::solveTeachIk(ctx);
			if (!r.ok)
			{
				continue;
			}
			const double contCost = std::abs(qe - prevQe) * 0.01;
			const double score = r.residualTcpMm + contCost;
			if (score < bestResidual)
			{
				bestResidual = score;
				bestQe = qe;
				bestOk = true;
				bestQ = r.jointRad;
			}
		}

		if (bestOk && allowCoupledRefine)
		{
			RobotTeachIk::TeachIkContext ctx;
			ctx.urdfPath = m_urdfPath;
			ctx.ikLinkName = m_ikLinkName;
			ctx.seedJointRad = bestQ;
			ctx.useOrientation = false;
			ctx.maxIkIterations = 60;
			ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(tp.poseMm.x, tp.poseMm.y, tp.poseMm.z,
																				 tp.eulerDeg.x, tp.eulerDeg.y, tp.eulerDeg.z);
			ctx.externalAxis.enabled = true;
			ctx.externalAxis.isPrismatic = cfg->isPrismatic;
			ctx.externalAxis.axis[0] = cfg->axis[0];
			ctx.externalAxis.axis[1] = cfg->axis[1];
			ctx.externalAxis.axis[2] = cfg->axis[2];
			ctx.externalAxis.lower = cfg->lower;
			ctx.externalAxis.upper = cfg->upper;
			ctx.externalAxis.qExternal = bestQe;
			ctx.externalAxis.optimizeExternal = true;
			const RobotTeachIk::TeachIkResult refined = RobotTeachIk::solveTeachIk(ctx);
			if (refined.ok)
			{
				bestQe = refined.externalAxisQ;
				bestQ = refined.jointRad;
				bestResidual = refined.residualTcpMm;
			}
		}

		tp.reachable = bestOk && bestResidual < 5.0;
		snap.positionMmOrRad = bestOk ? bestQe : cfg->home;
		prevQe = snap.positionMmOrRad;
		if (bestOk && !bestQ.empty())
		{
			seed = bestQ;
		}
	}

	traj.ctx.externalAxes.push_back(snap);
	return true;
}

} // namespace RobotInstruction
