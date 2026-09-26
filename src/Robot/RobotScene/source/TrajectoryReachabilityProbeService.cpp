/// @file TrajectoryReachabilityProbeService.cpp
/// @brief TeachIk 逐点探测；成功点滚动关节种子

#include "TrajectoryReachabilityProbeService.h"

#include "RobotTeachIk.h"
#include "UnifiedTrajectory.h"

#include <IkAcceptanceGates.h>

namespace RobotInstruction
{
namespace
{
ReachabilityProbeLastStats g_lastReachabilityStats{};
}

ReachabilityProbeLastStats trajectoryReachabilityLastStats()
{
	return g_lastReachabilityStats;
}

void resetTrajectoryReachabilityLastStats()
{
	g_lastReachabilityStats = ReachabilityProbeLastStats{};
}

void TrajectoryReachabilityProbeService::setRobotContext(const QString& urdfPath, const QString& ikLinkName,
														 const std::vector<double>& seedJointRad,
														 const BackendMat4& T_flange_tool, const bool useOrientation,
														 const bool allowApproximateOrientation)
{
	m_urdfPath = urdfPath;
	m_ikLinkName = ikLinkName;
	m_seedJointRad = seedJointRad;
	m_T_flange_tool = T_flange_tool;
	m_useOrientation = useOrientation;
	m_allowApproximateOrientation = allowApproximateOrientation;
}

bool TrajectoryReachabilityProbeService::probe(UnifiedTrajectory& traj, const std::vector<std::size_t>& indices,
											   const bool useOrientation, const double residualTolMm,
											   std::string* errMsg) const
{
	resetTrajectoryReachabilityLastStats();
	if (m_urdfPath.isEmpty() || m_ikLinkName.isEmpty() || m_seedJointRad.empty())
	{
		if (errMsg)
		{
			*errMsg = "reachability probe requires URDF/IK context";
		}
		return false;
	}

	std::vector<std::size_t> order = indices;
	if (order.empty())
	{
		order.resize(traj.points.size());
		for (std::size_t i = 0; i < traj.points.size(); ++i)
		{
			order[i] = i;
		}
	}

	using Gates = UrdfRobotLoader::IkAcceptanceGates;
	const double defaultTol = Gates::acceptPosMm(m_allowApproximateOrientation);
	const double tol = residualTolMm > 0.0 ? residualTolMm : defaultTol;
	const double oriGate = Gates::acceptOrientDeg(m_allowApproximateOrientation);
	const bool oriWanted = useOrientation && m_useOrientation;
	std::vector<double> seed = m_seedJointRad;
	std::size_t probed = 0;
	std::size_t unreachable = 0;

	for (const std::size_t idx : order)
	{
		if (idx >= traj.points.size())
		{
			continue;
		}
		UnifiedTrajectoryPoint& tp = traj.points[idx];
		RobotTeachIk::TeachIkContext ctx;
		ctx.urdfPath = m_urdfPath;
		ctx.ikLinkName = m_ikLinkName;
		ctx.seedJointRad = seed;
		ctx.useOrientation = oriWanted;
		ctx.T_flange_tool = m_T_flange_tool;
		ctx.options.allowApproximateOrientation = m_allowApproximateOrientation;
		ctx.maxIkIterations = 80;
		ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(
			tp.poseMm.x, tp.poseMm.y, tp.poseMm.z, tp.eulerDeg.x, tp.eulerDeg.y, tp.eulerDeg.z);

		const RobotTeachIk::TeachIkResult r = RobotTeachIk::solveTeachIk(ctx);
		const bool orientOk = !oriWanted || r.residualOrientDeg < 0.0 || r.residualOrientDeg <= oriGate;
		tp.reachable = r.ok && r.residualTcpMm >= 0.0 && r.residualTcpMm < tol && orientOk;
		++probed;
		if (!tp.reachable)
		{
			++unreachable;
		}
		if (r.ok && !r.jointRad.empty())
		{
			seed = r.jointRad;
		}
	}

	g_lastReachabilityStats.valid = true;
	g_lastReachabilityStats.probedCount = probed;
	g_lastReachabilityStats.unreachableCount = unreachable;
	return true;
}

} // namespace RobotInstruction
