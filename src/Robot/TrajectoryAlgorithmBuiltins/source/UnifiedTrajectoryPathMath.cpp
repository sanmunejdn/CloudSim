/// @file UnifiedTrajectoryPathMath.cpp
/// @brief 统一轨迹路径数学

// UnifiedTrajectoryPathMath 实现
#include "UnifiedTrajectoryPathMath.h"

#include "RigidTransform.h"
#include "TrajectoryTransformMath.h"
#include "TrajectoryUnifiedScope.h"
#include "UnifiedTrajectorySemanticMath.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <vector>

namespace trajectory_algo
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double dist3(const RobotInstruction::Vec3& a, const RobotInstruction::Vec3& b)
{
	const double dx = b.x - a.x;
	const double dy = b.y - a.y;
	const double dz = b.z - a.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void replaceScopedBlock(RobotInstruction::UnifiedTrajectory& traj, const std::vector<std::size_t>& indices,
						std::vector<RobotInstruction::UnifiedTrajectoryPoint>&& replacement)
{
	if (indices.empty())
	{
		return;
	}
	const std::size_t insertAt = indices.front();
	traj.points.erase(traj.points.begin() + static_cast<std::ptrdiff_t>(insertAt),
					  traj.points.begin() + static_cast<std::ptrdiff_t>(insertAt + indices.size()));
	traj.points.insert(traj.points.begin() + static_cast<std::ptrdiff_t>(insertAt), replacement.begin(),
					   replacement.end());
}

void applyEulerFromAxes(RobotInstruction::UnifiedTrajectoryPoint& tp, Eigen::Vector3d xAxis, Eigen::Vector3d zAxis)
{
	if (zAxis.norm() < 1e-12)
	{
		zAxis = Eigen::Vector3d::UnitZ();
	}
	zAxis.normalize();
	xAxis = xAxis - zAxis * xAxis.dot(zAxis);
	if (xAxis.norm() < 1e-12)
	{
		xAxis = zAxis.unitOrthogonal();
	}
	xAxis.normalize();
	Eigen::Vector3d yAxis = zAxis.cross(xAxis).normalized();
	Eigen::Matrix3d rot;
	rot.col(0) = xAxis;
	rot.col(1) = yAxis;
	rot.col(2) = zAxis;
	const engine::RigidTransform tf = engine::RigidTransform::fromTranslationQuat(
		Eigen::Vector3d(tp.poseMm.x, tp.poseMm.y, tp.poseMm.z), Eigen::Quaterniond(rot));
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	tf.eulerDegForDisplay(ex, ey, ez);
	tp.eulerDeg = RobotInstruction::Vec3{ex, ey, ez};
}

Eigen::Vector3d weavePoseUnitAxis(const int poseAxis)
{
	switch (poseAxis)
	{
	case 1:
		return Eigen::Vector3d::UnitY();
	case 2:
		return Eigen::Vector3d::UnitZ();
	case 0:
	default:
		return Eigen::Vector3d::UnitX();
	}
}

// 物体系：位置沿工具 Y 正弦侧移；可选绕工具轴姿态摆
void applyWeaveAtPhase(RobotInstruction::UnifiedTrajectoryPoint& tp, const double phase,
					   const RobotInstruction::WeaveParams& params)
{
	const double s = std::sin(phase);
	engine::RigidTransform tf = rigidFromPoint(tp);
	if (std::abs(params.amplitudeMm) > 1e-12)
	{
		const engine::RigidTransform posDelta = engine::RigidTransform::fromTranslationQuat(
			Eigen::Vector3d(0.0, params.amplitudeMm * s, 0.0), Eigen::Quaterniond::Identity());
		tf = applyTransformDelta(tf, posDelta, RobotInstruction::TransformReferenceFrame::Body);
	}
	if (params.wavePose && std::abs(params.poseAmplitudeDeg) > 1e-12)
	{
		const double rad = params.poseAmplitudeDeg * s * kPi / 180.0;
		const engine::RigidTransform rotDelta = engine::RigidTransform::fromTranslationQuat(
			Eigen::Vector3d::Zero(), Eigen::Quaterniond(Eigen::AngleAxisd(rad, weavePoseUnitAxis(params.poseAxis))));
		tf = applyTransformDelta(tf, rotDelta, RobotInstruction::TransformReferenceFrame::Body);
	}
	pointFromRigid(tf, tp);
}

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

void offsetLateralUnified(RobotInstruction::UnifiedTrajectory& traj, const double lateralMm)
{
	// 航向水平左向（仅 yaw），Z 不变
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

void weaveUnified(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::WeaveParams& params)
{
	if (params.periodMm <= 1e-6)
	{
		return;
	}
	double acc = 0.0;
	for (std::size_t i = 0; i < traj.points.size(); ++i)
	{
		if (i > 0U)
		{
			acc += dist3(traj.points[i - 1U].poseMm, traj.points[i].poseMm);
		}
		applyWeaveAtPhase(traj.points[i], (acc / params.periodMm) * 2.0 * kPi, params);
	}
}

bool reachabilityFilterUnified(RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
							   const bool useOrientation, const double residualTolMm, std::string* errMsg)
{
	if (!ctx.reachabilityProbe)
	{
		if (errMsg)
		{
			*errMsg = "reachability probe not injected";
		}
		return false;
	}
	return ctx.reachabilityProbe->probe(traj, {}, useOrientation, residualTolMm, errMsg);
}

void externalAxisSearchUnified(RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx)
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

void offsetLateralUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								 const RobotInstruction::RobotProgram* program, const double lateralMm)
{
	forEachScopedPoint(traj, scope, program,
					   [lateralMm](RobotInstruction::UnifiedTrajectoryPoint& tp)
					   {
						   // 航向水平左向（仅 yaw），Z 不变
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
						 const RobotInstruction::RobotProgram* program, const RobotInstruction::WeaveParams& params)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.empty() || params.periodMm <= 1e-6)
	{
		return;
	}
	double acc = 0.0;
	for (std::size_t k = 0; k < indices.size(); ++k)
	{
		const std::size_t i = indices[k];
		if (k > 0U)
		{
			acc += dist3(traj.points[indices[k - 1U]].poseMm, traj.points[i].poseMm);
		}
		applyWeaveAtPhase(traj.points[i], (acc / params.periodMm) * 2.0 * kPi, params);
	}
}

bool reachabilityFilterUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
									  const RobotInstruction::RobotProgram* program,
									  const TrajectoryOpExecutionContext& ctx, const bool useOrientation,
									  const double residualTolMm, std::string* errMsg)
{
	if (!ctx.reachabilityProbe)
	{
		if (errMsg)
		{
			*errMsg = "reachability probe not injected";
		}
		return false;
	}
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	return ctx.reachabilityProbe->probe(traj, indices, useOrientation, residualTolMm, errMsg);
}

void reversePathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.size() < 2U)
	{
		return;
	}
	for (std::size_t a = 0, b = indices.size() - 1U; a < b; ++a, --b)
	{
		std::swap(traj.points[indices[a]], traj.points[indices[b]]);
	}
}

void orientToPathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								const RobotInstruction::RobotProgram* program, const bool flipToolZ)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.empty())
	{
		return;
	}
	for (std::size_t k = 0; k < indices.size(); ++k)
	{
		const std::size_t i = indices[k];
		Eigen::Vector3d tangent = Eigen::Vector3d::UnitX();
		if (indices.size() == 1U)
		{
			// 单点：保留切向兜底
		}
		else if (k + 1U < indices.size())
		{
			const auto& a = traj.points[i].poseMm;
			const auto& b = traj.points[indices[k + 1U]].poseMm;
			tangent = Eigen::Vector3d(b.x - a.x, b.y - a.y, b.z - a.z);
		}
		else
		{
			const auto& a = traj.points[indices[k - 1U]].poseMm;
			const auto& b = traj.points[i].poseMm;
			tangent = Eigen::Vector3d(b.x - a.x, b.y - a.y, b.z - a.z);
		}
		if (tangent.norm() < 1e-12)
		{
			tangent = Eigen::Vector3d::UnitX();
		}
		tangent.normalize();
		Eigen::Vector3d zAxis = Eigen::Vector3d::UnitZ();
		if (flipToolZ)
		{
			zAxis = -zAxis;
		}
		applyEulerFromAxes(traj.points[i], tangent, zAxis);
	}
}

void trimPathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							const RobotInstruction::RobotProgram* program, const double startMm, const double endMm)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.size() < 2U || (startMm <= 0.0 && endMm <= 0.0))
	{
		return;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> sub;
	sub.reserve(indices.size());
	for (const std::size_t idx : indices)
	{
		sub.push_back(traj.points[idx]);
	}
	std::vector<double> cum(sub.size(), 0.0);
	for (std::size_t i = 1; i < sub.size(); ++i)
	{
		cum[i] = cum[i - 1U] + dist3(sub[i - 1U].poseMm, sub[i].poseMm);
	}
	const double total = cum.back();
	const double keepStart = std::max(0.0, startMm);
	const double keepEnd = std::max(0.0, total - endMm);
	if (keepEnd <= keepStart + 1e-9)
	{
		return;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> kept;
	kept.reserve(sub.size());
	for (std::size_t i = 0; i < sub.size(); ++i)
	{
		if (cum[i] + 1e-9 >= keepStart && cum[i] - 1e-9 <= keepEnd)
		{
			kept.push_back(sub[i]);
		}
	}
	if (kept.size() < 2U)
	{
		return;
	}
	replaceScopedBlock(traj, indices, std::move(kept));
}

void shiftStartUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							  const RobotInstruction::RobotProgram* program, const int startIndex1)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.size() < 2U)
	{
		return;
	}
	const int n = static_cast<int>(indices.size());
	int start0 = startIndex1 - 1;
	if (start0 < 0)
	{
		start0 = 0;
	}
	start0 %= n;
	if (start0 == 0)
	{
		return;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> rotated;
	rotated.reserve(indices.size());
	for (int k = 0; k < n; ++k)
	{
		rotated.push_back(traj.points[indices[static_cast<std::size_t>((start0 + k) % n)]]);
	}
	replaceScopedBlock(traj, indices, std::move(rotated));
}

void insertMidpointsUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								   const RobotInstruction::RobotProgram* program, const int passes)
{
	if (passes < 1)
	{
		return;
	}
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.size() < 2U)
	{
		return;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> cur;
	cur.reserve(indices.size());
	for (const std::size_t idx : indices)
	{
		cur.push_back(traj.points[idx]);
	}
	for (int pass = 0; pass < passes; ++pass)
	{
		std::vector<RobotInstruction::UnifiedTrajectoryPoint> next;
		next.reserve(cur.size() * 2U);
		for (std::size_t i = 0; i + 1U < cur.size(); ++i)
		{
			next.push_back(cur[i]);
			RobotInstruction::UnifiedTrajectoryPoint mid = cur[i];
			mid.poseMm.x = 0.5 * (cur[i].poseMm.x + cur[i + 1U].poseMm.x);
			mid.poseMm.y = 0.5 * (cur[i].poseMm.y + cur[i + 1U].poseMm.y);
			mid.poseMm.z = 0.5 * (cur[i].poseMm.z + cur[i + 1U].poseMm.z);
			mid.eulerDeg.x = 0.5 * (cur[i].eulerDeg.x + cur[i + 1U].eulerDeg.x);
			mid.eulerDeg.y = 0.5 * (cur[i].eulerDeg.y + cur[i + 1U].eulerDeg.y);
			mid.eulerDeg.z = 0.5 * (cur[i].eulerDeg.z + cur[i + 1U].eulerDeg.z);
			mid.speedMmPerSec = 0.5 * (cur[i].speedMmPerSec + cur[i + 1U].speedMmPerSec);
			mid.blendRadiusMm = 0.5 * (cur[i].blendRadiusMm + cur[i + 1U].blendRadiusMm);
			mid.sourceInstructionId.clear();
			next.push_back(mid);
		}
		next.push_back(cur.back());
		cur = std::move(next);
	}
	replaceScopedBlock(traj, indices, std::move(cur));
}

void clampToPlaneUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								const RobotInstruction::RobotProgram* program, const int axis, const double valueMm)
{
	forEachScopedPoint(traj, scope, program,
					   [axis, valueMm](RobotInstruction::UnifiedTrajectoryPoint& tp)
					   {
						   if (axis == 0)
						   {
							   tp.poseMm.x = valueMm;
						   }
						   else if (axis == 1)
						   {
							   tp.poseMm.y = valueMm;
						   }
						   else
						   {
							   tp.poseMm.z = valueMm;
						   }
					   });
}

void scalePathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							 const RobotInstruction::RobotProgram* program, const double factor)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.empty() || std::abs(factor - 1.0) < 1e-15)
	{
		return;
	}
	const RobotInstruction::Vec3 origin = traj.points[indices.front()].poseMm;
	for (const std::size_t idx : indices)
	{
		auto& p = traj.points[idx].poseMm;
		p.x = origin.x + (p.x - origin.x) * factor;
		p.y = origin.y + (p.y - origin.y) * factor;
		p.z = origin.z + (p.z - origin.z) * factor;
	}
}

void speedRampUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							 const RobotInstruction::RobotProgram* program, const double startMmPerSec,
							 const double endMmPerSec)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.empty())
	{
		return;
	}
	if (indices.size() == 1U)
	{
		traj.points[indices.front()].speedMmPerSec = startMmPerSec;
		return;
	}
	const double denom = static_cast<double>(indices.size() - 1U);
	for (std::size_t k = 0; k < indices.size(); ++k)
	{
		const double t = static_cast<double>(k) / denom;
		traj.points[indices[k]].speedMmPerSec = startMmPerSec + (endMmPerSec - startMmPerSec) * t;
	}
}

void spikeFilterUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program, const double maxDetourRatio)
{
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	if (indices.size() < 3U || maxDetourRatio < 1.0)
	{
		return;
	}
	std::vector<RobotInstruction::UnifiedTrajectoryPoint> sub;
	sub.reserve(indices.size());
	for (const std::size_t idx : indices)
	{
		sub.push_back(traj.points[idx]);
	}
	bool removed = true;
	while (removed && sub.size() >= 3U)
	{
		removed = false;
		for (std::size_t i = 1; i + 1U < sub.size(); ++i)
		{
			const double dPrev = dist3(sub[i - 1U].poseMm, sub[i].poseMm);
			const double dNext = dist3(sub[i].poseMm, sub[i + 1U].poseMm);
			const double chord = dist3(sub[i - 1U].poseMm, sub[i + 1U].poseMm);
			if (chord > 1e-9 && (dPrev + dNext) / chord > maxDetourRatio)
			{
				sub.erase(sub.begin() + static_cast<std::ptrdiff_t>(i));
				removed = true;
				break;
			}
		}
	}
	if (sub.size() == indices.size())
	{
		return;
	}
	replaceScopedBlock(traj, indices, std::move(sub));
}

} // namespace trajectory_algo
