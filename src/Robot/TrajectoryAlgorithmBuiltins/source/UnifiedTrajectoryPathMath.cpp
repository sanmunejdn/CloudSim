/// @file UnifiedTrajectoryPathMath.cpp
/// @brief 统一轨迹路径数学

// UnifiedTrajectoryPathMath 实现
#include "UnifiedTrajectoryPathMath.h"

#include "TrajectoryUnifiedScope.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace trajectory_algo
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

} // namespace

void resampleUnifiedTrajectory(RobotInstruction::UnifiedTrajectory& traj, const double stepMm)
{
	if (traj.points.size() < 2U || stepMm <= 0.0)
	{
		return;
	}
	std::vector<double> segLen;
	double total = 0.0;
	for (std::size_t i = 1; i < traj.points.size(); ++i)
	{
		const auto& a = traj.points[i - 1U].poseMm;
		const auto& b = traj.points[i].poseMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
		segLen.push_back(len);
		total += len;
	}
	if (total < stepMm)
	{
		return;
	}
	const int n = std::max(2, static_cast<int>(std::ceil(total / stepMm)) + 1);
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> out;
	out.reserve(static_cast<std::size_t>(n));
	for (int s = 0; s < n; ++s)
	{
		const double t = static_cast<double>(s) / static_cast<double>(n - 1) * total;
		double acc = 0.0;
		std::size_t seg = 0;
		while (seg < segLen.size() && acc + segLen[seg] < t - 1e-9)
		{
			acc += segLen[seg];
			++seg;
		}
		const double local = (seg < segLen.size() && segLen[seg] > 1e-12) ? (t - acc) / segLen[seg] : 0.0;
		const auto& p0 = traj.points[seg].poseMm;
		const auto& p1 = traj.points[std::min(seg + 1U, traj.points.size() - 1U)].poseMm;
		RobotInstruction::UnifiedTrajectoryPoint tp = traj.points[seg];
		tp.poseMm.x = p0.x + (p1.x - p0.x) * local;
		tp.poseMm.y = p0.y + (p1.y - p0.y) * local;
		tp.poseMm.z = p0.z + (p1.z - p0.z) * local;
		tp.sourceInstructionId.clear();
		out.push_back(tp);
	}
	traj.points = std::move(out);
}

void offsetAlongNormalUnified(RobotInstruction::UnifiedTrajectory& traj, const double offsetMm)
{
	for (RobotInstruction::UnifiedTrajectoryPoint& tp : traj.points)
	{
		const double yaw = tp.eulerDeg.z * kPi / 180.0;
		const double pitch = tp.eulerDeg.y * kPi / 180.0;
		const RobotInstruction::Vec3 nz{std::sin(pitch) * std::cos(yaw), std::sin(pitch) * std::sin(yaw),
										std::cos(pitch)};
		tp.poseMm.x += nz.x * offsetMm;
		tp.poseMm.y += nz.y * offsetMm;
		tp.poseMm.z += nz.z * offsetMm;
	}
}

void offsetLateralUnified(RobotInstruction::UnifiedTrajectory& traj, const double lateralMm)
{
	for (RobotInstruction::UnifiedTrajectoryPoint& tp : traj.points)
	{
		const double yaw = tp.eulerDeg.z * kPi / 180.0;
		const RobotInstruction::Vec3 lateral{-std::sin(yaw), std::cos(yaw), 0.0};
		tp.poseMm.x += lateral.x * lateralMm;
		tp.poseMm.y += lateral.y * lateralMm;
	}
}

void smoothPoseUnified(RobotInstruction::UnifiedTrajectory& traj)
{
	if (traj.points.size() < 3U)
	{
		return;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> smoothed = traj.points;
	for (std::size_t i = 1; i + 1U < traj.points.size(); ++i)
	{
		smoothed[i].poseMm.x =
			(traj.points[i - 1U].poseMm.x + traj.points[i].poseMm.x + traj.points[i + 1U].poseMm.x) / 3.0;
		smoothed[i].poseMm.y =
			(traj.points[i - 1U].poseMm.y + traj.points[i].poseMm.y + traj.points[i + 1U].poseMm.y) / 3.0;
		smoothed[i].poseMm.z =
			(traj.points[i - 1U].poseMm.z + traj.points[i].poseMm.z + traj.points[i + 1U].poseMm.z) / 3.0;
	}
	traj.points = std::move(smoothed);
}

void assignBlendUnified(RobotInstruction::UnifiedTrajectory& traj, const double blendRadiusMm)
{
	for (RobotInstruction::UnifiedTrajectoryPoint& tp : traj.points)
	{
		tp.blendRadiusMm = blendRadiusMm;
	}
}

void assignSpeedUnified(RobotInstruction::UnifiedTrajectory& traj, const double speedMmPerSec)
{
	for (RobotInstruction::UnifiedTrajectoryPoint& tp : traj.points)
	{
		tp.speedMmPerSec = speedMmPerSec;
	}
}

void weaveUnified(RobotInstruction::UnifiedTrajectory& traj, const double amplitudeMm, const double periodMm)
{
	double acc = 0.0;
	for (std::size_t i = 0; i < traj.points.size(); ++i)
	{
		if (i > 0U)
		{
			const auto& a = traj.points[i - 1U].poseMm;
			const auto& b = traj.points[i].poseMm;
			acc += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
		}
		const double phase = (periodMm > 1e-6) ? (acc / periodMm) * 2.0 * kPi : 0.0;
		const double off = amplitudeMm * std::sin(phase);
		const double yaw = traj.points[i].eulerDeg.z * kPi / 180.0;
		traj.points[i].poseMm.x += -std::sin(yaw) * off;
		traj.points[i].poseMm.y += std::cos(yaw) * off;
	}
}

void reachabilityFilterUnified(RobotInstruction::UnifiedTrajectory& traj)
{
	for (RobotInstruction::UnifiedTrajectoryPoint& tp : traj.points)
	{
		tp.reachable = tp.poseMm.z > -5000.0;
	}
}

void externalAxisSearchUnified(RobotInstruction::UnifiedTrajectory& traj,
							   const TrajectoryOpExecutionContext& ctx)
{
	bool anyEnabled = false;
	for (const ExternalAxisSearchConfigDto& c : ctx.externalAxisConfigs)
	{
		if (c.enabled)
		{
			anyEnabled = true;
			break;
		}
	}
	if (!anyEnabled)
	{
		return;
	}
	if (!ctx.externalAxisSearch)
	{
		return;
	}
	(void)ctx.externalAxisSearch->search(traj, ctx.externalAxisConfigs, ctx.externalAxisAllowCoupledRefine, nullptr);
}

void resampleUnifiedTrajectoryInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
									  const RobotInstruction::RobotProgram* program, const double stepMm)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.size() < 2U)
	{
		return;
	}
	RobotInstruction::UnifiedTrajectory sub{};
	sub.points.reserve(indices.size());
	for (const std::size_t idx : indices)
	{
		sub.points.push_back(traj.points[idx]);
	}
	resampleUnifiedTrajectory(sub, stepMm);
	const std::size_t insertAt = indices.front();
	traj.points.erase(traj.points.begin() + static_cast<std::ptrdiff_t>(insertAt),
					  traj.points.begin() + static_cast<std::ptrdiff_t>(insertAt + indices.size()));
	traj.points.insert(traj.points.begin() + static_cast<std::ptrdiff_t>(insertAt), sub.points.begin(),
					   sub.points.end());
}

namespace
{
template <typename Fn>
void forEachScopedPoint(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
						const RobotInstruction::RobotProgram* program, Fn&& fn)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	for (const std::size_t idx : indices)
	{
		fn(traj.points[idx]);
	}
}

} // namespace

void offsetAlongNormalUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
									 const RobotInstruction::RobotProgram* program, const double offsetMm)
{
	forEachScopedPoint(traj, scope, program,
					   [offsetMm](RobotInstruction::UnifiedTrajectoryPoint& tp)
					   {
						   const double yaw = tp.eulerDeg.z * kPi / 180.0;
						   const double pitch = tp.eulerDeg.y * kPi / 180.0;
						   const RobotInstruction::Vec3 nz{std::sin(pitch) * std::cos(yaw),
														   std::sin(pitch) * std::sin(yaw), std::cos(pitch)};
						   tp.poseMm.x += nz.x * offsetMm;
						   tp.poseMm.y += nz.y * offsetMm;
						   tp.poseMm.z += nz.z * offsetMm;
					   });
}

void offsetLateralUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								 const RobotInstruction::RobotProgram* program, const double lateralMm)
{
	forEachScopedPoint(traj, scope, program,
					   [lateralMm](RobotInstruction::UnifiedTrajectoryPoint& tp)
					   {
						   const double yaw = tp.eulerDeg.z * kPi / 180.0;
						   const RobotInstruction::Vec3 lateral{-std::sin(yaw), std::cos(yaw), 0.0};
						   tp.poseMm.x += lateral.x * lateralMm;
						   tp.poseMm.y += lateral.y * lateralMm;
					   });
}

void smoothPoseUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							  const RobotInstruction::RobotProgram* program)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.size() < 3U)
	{
		return;
	}
	for (std::size_t k = 1; k + 1U < indices.size(); ++k)
	{
		const std::size_t i = indices[k];
		const std::size_t prev = indices[k - 1U];
		const std::size_t next = indices[k + 1U];
		traj.points[i].poseMm.x =
			(traj.points[prev].poseMm.x + traj.points[i].poseMm.x + traj.points[next].poseMm.x) / 3.0;
		traj.points[i].poseMm.y =
			(traj.points[prev].poseMm.y + traj.points[i].poseMm.y + traj.points[next].poseMm.y) / 3.0;
		traj.points[i].poseMm.z =
			(traj.points[prev].poseMm.z + traj.points[i].poseMm.z + traj.points[next].poseMm.z) / 3.0;
	}
}

void assignBlendUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program, const double blendRadiusMm)
{
	forEachScopedPoint(traj, scope, program,
					   [blendRadiusMm](RobotInstruction::UnifiedTrajectoryPoint& tp)
					   { tp.blendRadiusMm = blendRadiusMm; });
}

void assignSpeedUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program, const double speedMmPerSec)
{
	forEachScopedPoint(traj, scope, program,
					   [speedMmPerSec](RobotInstruction::UnifiedTrajectoryPoint& tp)
					   { tp.speedMmPerSec = speedMmPerSec; });
}

void weaveUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
						 const RobotInstruction::RobotProgram* program, const double amplitudeMm, const double periodMm)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.empty())
	{
		return;
	}
	double acc = 0.0;
	for (std::size_t k = 0; k < indices.size(); ++k)
	{
		const std::size_t i = indices[k];
		if (k > 0U)
		{
			const std::size_t prev = indices[k - 1U];
			const auto& a = traj.points[prev].poseMm;
			const auto& b = traj.points[i].poseMm;
			acc += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
		}
		const double phase = (periodMm > 1e-6) ? (acc / periodMm) * 2.0 * kPi : 0.0;
		const double off = amplitudeMm * std::sin(phase);
		const double yaw = traj.points[i].eulerDeg.z * kPi / 180.0;
		traj.points[i].poseMm.x += -std::sin(yaw) * off;
		traj.points[i].poseMm.y += std::cos(yaw) * off;
	}
}

void reachabilityFilterUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
									  const RobotInstruction::RobotProgram* program)
{
	forEachScopedPoint(traj, scope, program,
					   [](RobotInstruction::UnifiedTrajectoryPoint& tp) { tp.reachable = tp.poseMm.z > -5000.0; });
}

} // namespace trajectory_algo
