/// @file ExternalAxisSearchService.cpp
/// @brief 外部轴网格搜索：多轴启用时走 TeachIk 采样+可选联立

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
	std::vector<const trajectory_algo::ExternalAxisSearchConfigDto*> enabled;
	for (const auto& c : configs)
	{
		if (c.enabled)
		{
			enabled.push_back(&c);
		}
	}
	if (enabled.empty())
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

	std::vector<double> seed = m_seedJointRad;
	traj.ctx.externalAxes.clear();
	std::vector<ExternalAxisSnapshot> snaps;
	snaps.reserve(enabled.size());
	for (const auto* cfg : enabled)
	{
		ExternalAxisSnapshot snap;
		snap.jointName = cfg->jointName.empty() ? "rail_joint" : cfg->jointName;
		snap.isPrismatic = cfg->isPrismatic;
		snap.positionMmOrRad = cfg->home;
		snaps.push_back(snap);
	}

	auto fillDof = [&](RobotTeachIk::TeachIkExternalAxisDof& dof, const bool optimize) {
		dof = {};
		dof.optimizeExternal = optimize;
		dof.adaptiveExternalDamping = true;
		for (size_t i = 0; i < enabled.size(); ++i)
		{
			const auto* cfg = enabled[i];
			RobotTeachIk::TeachIkExternalAxisSlot slot;
			slot.configIndex = static_cast<int>(i);
			slot.isPrismatic = cfg->isPrismatic;
			slot.axis[0] = cfg->axis[0];
			slot.axis[1] = cfg->axis[1];
			slot.axis[2] = cfg->axis[2];
			slot.lower = cfg->lower;
			slot.upper = cfg->upper;
			dof.axes.push_back(slot);
			dof.qExternal.push_back(snaps[i].positionMmOrRad);
		}
	};

	for (UnifiedTrajectoryPoint& tp : traj.points)
	{
		RobotTeachIk::TeachIkContext ctx;
		ctx.urdfPath = m_urdfPath;
		ctx.ikLinkName = m_ikLinkName;
		ctx.seedJointRad = seed;
		ctx.useOrientation = false;
		ctx.maxIkIterations = 80;
		ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(tp.poseMm.x, tp.poseMm.y, tp.poseMm.z,
																			 tp.eulerDeg.x, tp.eulerDeg.y, tp.eulerDeg.z);
		ctx.externalAxisConfigCount = static_cast<int>(enabled.size());
		fillDof(ctx.externalAxes, false);
		RobotTeachIk::TeachIkResult r = RobotTeachIk::solveTeachIk(ctx);
		if (r.ok && allowCoupledRefine && enabled.size() <= 2)
		{
			ctx.seedJointRad = r.jointRad;
			fillDof(ctx.externalAxes, true);
			for (size_t i = 0; i < ctx.externalAxes.qExternal.size() && i < r.externalAxisQs.size(); ++i)
			{
				ctx.externalAxes.qExternal[i] = r.externalAxisQs[i];
			}
			ctx.maxIkIterations = 60;
			const RobotTeachIk::TeachIkResult refined = RobotTeachIk::solveTeachIk(ctx);
			if (refined.ok)
			{
				r = refined;
			}
		}

		tp.reachable = r.ok && r.residualTcpMm < 5.0;
		if (r.ok)
		{
			for (size_t i = 0; i < snaps.size(); ++i)
			{
				if (i < r.externalAxisQs.size())
				{
					snaps[i].positionMmOrRad = r.externalAxisQs[i];
				}
			}
			if (!r.jointRad.empty())
			{
				seed = r.jointRad;
			}
		}
	}

	for (const ExternalAxisSnapshot& s : snaps)
	{
		traj.ctx.externalAxes.push_back(s);
	}
	return true;
}

} // namespace RobotInstruction
