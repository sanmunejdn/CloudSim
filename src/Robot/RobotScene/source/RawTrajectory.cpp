#include "RawTrajectory.h"

#include "GeometryRef.h"
#include "RobotInstructionTransform.h"
#include "RobotProgramCatalog.h"
#include "RobotInstructionProgram.h"

#include <FeatureSpec.h>
#include <RigidTransform.h>

#include <json.hpp>

#include <cmath>
#include <random>
#include <unordered_set>

namespace RobotInstruction
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Vec3 normalizeVec(const Vec3& v)
{
	const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (len < 1e-12)
	{
		return Vec3{0.0, 0.0, 1.0};
	}
	return Vec3{v.x / len, v.y / len, v.z / len};
}

Vec3 crossVec(const Vec3& a, const Vec3& b)
{
	return Vec3{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x};
}

Vec3 eulerFromFrame(const Vec3& zAxis, const Vec3& xHint)
{
	const Vec3 z = normalizeVec(zAxis);
	Vec3 x = xHint;
	const double dot = x.x * z.x + x.y * z.y + x.z * z.z;
	x.x -= dot * z.x;
	x.y -= dot * z.y;
	x.z -= dot * z.z;
	x = normalizeVec(x);
	const Vec3 y = crossVec(z, x);
	(void)y;
	const double sy = std::sqrt(x.x * x.x + x.y * x.y);
	const bool singular = sy < 1e-6;
	double yaw = 0.0;
	double pitch = 0.0;
	double roll = 0.0;
	if (!singular)
	{
		yaw = std::atan2(x.y, x.x) * 180.0 / kPi;
		pitch = std::atan2(-x.z, sy) * 180.0 / kPi;
		roll = std::atan2(z.x * x.y - z.y * x.x, z.x * x.x + z.y * x.y) * 180.0 / kPi;
	}
	else
	{
		yaw = std::atan2(-z.y, z.x) * 180.0 / kPi;
		pitch = std::atan2(-x.z, sy) * 180.0 / kPi;
	}
	return Vec3{roll, pitch, yaw};
}

void resampleTrajectory(RawTrajectory& traj, double stepMm)
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
	std::vector<TrajectoryPoint> out;
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
		TrajectoryPoint tp = traj.points[seg];
		tp.poseMm.x = p0.x + (p1.x - p0.x) * local;
		tp.poseMm.y = p0.y + (p1.y - p0.y) * local;
		tp.poseMm.z = p0.z + (p1.z - p0.z) * local;
		out.push_back(tp);
	}
	traj.points = std::move(out);
}

} // namespace

bool importRawPathToTrajectory(
	const geoalgo::RawPath& path,
	FrameStrategy strategy,
	RawTrajectory& out,
	std::string* errMsg)
{
	if (path.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty raw path";
		}
		return false;
	}
	out = RawTrajectory{};
	out.sourceFeatureJson = geometry_backend_ops::featureSpecToJson(path.sourceSpec);
	out.points.reserve(path.points.size());
	for (std::size_t i = 0; i < path.points.size(); ++i)
	{
		const geoalgo::RawPathPoint& rp = path.points[i];
		TrajectoryPoint tp;
		tp.poseMm = Vec3{rp.positionMm.x, rp.positionMm.y, rp.positionMm.z};
		Vec3 zAxis{0.0, 0.0, 1.0};
		Vec3 xHint{1.0, 0.0, 0.0};
		if (strategy == FrameStrategy::SurfaceNormalZ && rp.hasNormal)
		{
			zAxis = Vec3{rp.normal.x, rp.normal.y, rp.normal.z};
		}
		if (rp.hasTangent)
		{
			xHint = Vec3{rp.tangent.x, rp.tangent.y, rp.tangent.z};
		}
		else if (i + 1U < path.points.size())
		{
			const auto& n = path.points[i + 1U].positionMm;
			xHint = Vec3{n.x - rp.positionMm.x, n.y - rp.positionMm.y, n.z - rp.positionMm.z};
		}
		tp.eulerDeg = eulerFromFrame(zAxis, xHint);
		out.points.push_back(tp);
	}
	return true;
}

bool applyRawTrajectoryOp(const RawTrajectoryOpDescriptor& op, RawTrajectory& trajectory, std::string* errMsg)
{
	switch (op.kind)
	{
	case RawTrajectoryOpKind::Resample:
		resampleTrajectory(trajectory, op.stepMm);
		return true;
	case RawTrajectoryOpKind::OffsetAlongNormal:
		for (TrajectoryPoint& tp : trajectory.points)
		{
			Vec3 z = normalizeVec(Vec3{tp.eulerDeg.x, tp.eulerDeg.y, tp.eulerDeg.z});
			(void)z;
			const double r = op.offsetMm;
			const double yaw = tp.eulerDeg.z * kPi / 180.0;
			const double pitch = tp.eulerDeg.y * kPi / 180.0;
			const Vec3 nz{
				std::sin(pitch) * std::cos(yaw),
				std::sin(pitch) * std::sin(yaw),
				std::cos(pitch)};
			tp.poseMm.x += nz.x * r;
			tp.poseMm.y += nz.y * r;
			tp.poseMm.z += nz.z * r;
		}
		return true;
	case RawTrajectoryOpKind::OffsetLateral:
		for (TrajectoryPoint& tp : trajectory.points)
		{
			const double yaw = tp.eulerDeg.z * kPi / 180.0;
			const Vec3 lateral{-std::sin(yaw), std::cos(yaw), 0.0};
			tp.poseMm.x += lateral.x * op.lateralMm;
			tp.poseMm.y += lateral.y * op.lateralMm;
		}
		return true;
	case RawTrajectoryOpKind::SmoothPose:
		if (trajectory.points.size() >= 3U)
		{
			std::vector<TrajectoryPoint> smoothed = trajectory.points;
			for (std::size_t i = 1; i + 1U < trajectory.points.size(); ++i)
			{
				smoothed[i].poseMm.x = (trajectory.points[i - 1U].poseMm.x + trajectory.points[i].poseMm.x
					+ trajectory.points[i + 1U].poseMm.x)
					/ 3.0;
				smoothed[i].poseMm.y = (trajectory.points[i - 1U].poseMm.y + trajectory.points[i].poseMm.y
					+ trajectory.points[i + 1U].poseMm.y)
					/ 3.0;
				smoothed[i].poseMm.z = (trajectory.points[i - 1U].poseMm.z + trajectory.points[i].poseMm.z
					+ trajectory.points[i + 1U].poseMm.z)
					/ 3.0;
			}
			trajectory.points = std::move(smoothed);
		}
		return true;
	case RawTrajectoryOpKind::AssignBlend:
		for (TrajectoryPoint& tp : trajectory.points)
		{
			tp.blendRadiusMm = op.blendRadiusMm;
		}
		return true;
	case RawTrajectoryOpKind::AssignSpeedZone:
		for (TrajectoryPoint& tp : trajectory.points)
		{
			tp.speedMmPerSec = op.speedMmPerSec;
		}
		return true;
	case RawTrajectoryOpKind::Weave:
	{
		double acc = 0.0;
		for (std::size_t i = 0; i < trajectory.points.size(); ++i)
		{
			if (i > 0U)
			{
				const auto& a = trajectory.points[i - 1U].poseMm;
				const auto& b = trajectory.points[i].poseMm;
				acc += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
			}
			const double phase = (op.weavePeriodMm > 1e-6) ? (acc / op.weavePeriodMm) * 2.0 * kPi : 0.0;
			const double off = op.weaveAmplitudeMm * std::sin(phase);
			const double yaw = trajectory.points[i].eulerDeg.z * kPi / 180.0;
			trajectory.points[i].poseMm.x += -std::sin(yaw) * off;
			trajectory.points[i].poseMm.y += std::cos(yaw) * off;
		}
		return true;
	}
	case RawTrajectoryOpKind::InsertApproachRetract:
		if (!trajectory.points.empty())
		{
			TrajectoryPoint approach = trajectory.points.front();
			const Vec3 nz = normalizeVec(Vec3{approach.eulerDeg.x, approach.eulerDeg.y, approach.eulerDeg.z});
			(void)nz;
			approach.poseMm.z += op.approachMm;
			trajectory.points.insert(trajectory.points.begin(), approach);
			TrajectoryPoint retract = trajectory.points.back();
			retract.poseMm.z += op.retractMm;
			trajectory.points.push_back(retract);
		}
		return true;
	case RawTrajectoryOpKind::ReachabilityFilter:
		for (TrajectoryPoint& tp : trajectory.points)
		{
			tp.reachable = tp.poseMm.z > -5000.0;
		}
		return true;
	case RawTrajectoryOpKind::ExternalAxisSearch:
		if (trajectory.ctx.externalAxes.empty())
		{
			ExternalAxisSnapshot rail;
			rail.jointName = "rail_joint";
			rail.isPrismatic = true;
			rail.positionMmOrRad = trajectory.points.empty() ? 0.0 : trajectory.points.front().poseMm.x * 0.1;
			trajectory.ctx.externalAxes.push_back(rail);
		}
		return true;
	case RawTrajectoryOpKind::FrameFromPath:
		return true;
	case RawTrajectoryOpKind::EmitToProgram:
		if (errMsg)
		{
			*errMsg = "EmitToProgram must use emitRawTrajectoryToProgram";
		}
		return false;
	}
	if (errMsg)
	{
		*errMsg = "unknown raw trajectory op";
	}
	return false;
}

bool applyRawTrajectoryPipeline(
	const std::vector<RawTrajectoryOpDescriptor>& ops,
	RawTrajectory& trajectory,
	std::string* errMsg)
{
	for (const RawTrajectoryOpDescriptor& op : ops)
	{
		if (op.kind == RawTrajectoryOpKind::EmitToProgram)
		{
			continue;
		}
		if (!applyRawTrajectoryOp(op, trajectory, errMsg))
		{
			return false;
		}
	}
	return true;
}

std::vector<RawTrajectoryOpDescriptor> rawTrajectoryRecipeWeldDefault()
{
	return {
		{RawTrajectoryOpKind::FrameFromPath, FrameStrategy::SurfaceNormalZ},
		{RawTrajectoryOpKind::Resample, FrameStrategy::SurfaceNormalZ, 5.0},
		{RawTrajectoryOpKind::OffsetAlongNormal, FrameStrategy::SurfaceNormalZ, 0.0, 0.0},
		{RawTrajectoryOpKind::SmoothPose},
		{RawTrajectoryOpKind::AssignBlend, FrameStrategy::SurfaceNormalZ, 0.0, 0.0, 0.0, 2.0},
		{RawTrajectoryOpKind::InsertApproachRetract, FrameStrategy::SurfaceNormalZ, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 20.0},
		{RawTrajectoryOpKind::ReachabilityFilter},
		{RawTrajectoryOpKind::EmitToProgram, FrameStrategy::SurfaceNormalZ, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true},
	};
}

std::vector<RawTrajectoryOpDescriptor> rawTrajectoryRecipeGlueDefault()
{
	return {
		{RawTrajectoryOpKind::FrameFromPath, FrameStrategy::SurfaceNormalZ},
		{RawTrajectoryOpKind::Resample, FrameStrategy::SurfaceNormalZ, 2.0},
		{RawTrajectoryOpKind::OffsetAlongNormal, FrameStrategy::SurfaceNormalZ, 0.0, 5.0},
		{RawTrajectoryOpKind::OffsetLateral, FrameStrategy::SurfaceNormalZ, 0.0, 0.0, 3.0},
		{RawTrajectoryOpKind::AssignSpeedZone, FrameStrategy::SurfaceNormalZ, 0.0, 0.0, 0.0, 0.0, 50.0},
		{RawTrajectoryOpKind::EmitToProgram},
	};
}

std::vector<RawTrajectoryOpDescriptor> rawTrajectoryRecipeGrindDefault()
{
	return {
		{RawTrajectoryOpKind::FrameFromPath, FrameStrategy::SurfaceNormalZ},
		{RawTrajectoryOpKind::SmoothPose},
		{RawTrajectoryOpKind::InsertApproachRetract},
		{RawTrajectoryOpKind::ReachabilityFilter},
		{RawTrajectoryOpKind::ExternalAxisSearch},
		{RawTrajectoryOpKind::EmitToProgram},
	};
}

bool emitRawTrajectoryToProgram(
	const RawTrajectory& trajectory,
	RobotProgram& program,
	std::string* errMsg,
	std::string* outGroupId,
	const std::string* pathPlanInstructionId)
{
	if (trajectory.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty trajectory";
		}
		return false;
	}
	if (pathPlanInstructionId && !pathPlanInstructionId->empty())
	{
		std::unordered_set<std::string> staleMotionIds;
		for (auto it = program.groups.begin(); it != program.groups.end();)
		{
			if (it->role == InstructionGroupRole::PathPlanOutput
				&& it->pathPlanInstructionId == *pathPlanInstructionId)
			{
				for (const std::string& id : it->memberInstructionIds)
				{
					staleMotionIds.insert(id);
				}
				it = program.groups.erase(it);
			}
			else
			{
				++it;
			}
		}
		program.steps.erase(
			std::remove_if(
				program.steps.begin(),
				program.steps.end(),
				[&staleMotionIds](const std::shared_ptr<Base>& ins) {
					return ins && staleMotionIds.count(ins->id()) != 0;
				}),
			program.steps.end());
	}
	else
	{
		program.steps.clear();
		program.groups.clear();
	}
	std::vector<std::string> memberIds;
	std::vector<std::shared_ptr<Base>> newMotion;
	memberIds.reserve(trajectory.points.size());
	newMotion.reserve(trajectory.points.size());
	int idx = 0;
	for (const TrajectoryPoint& tp : trajectory.points)
	{
		if (!tp.reachable)
		{
			continue;
		}
		auto ins = std::make_shared<LineInstruction>();
		ins->setName("P" + std::to_string(++idx));
		const engine::RigidTransform target = engine::RigidTransform::fromTranslationEulerDeg(
			tp.poseMm.x,
			tp.poseMm.y,
			tp.poseMm.z,
			tp.eulerDeg.x,
			tp.eulerDeg.y,
			tp.eulerDeg.z);
		writeTargetTransformToInstruction(*ins, target);
		ins->setBlendRadius(tp.blendRadiusMm);
		if (tp.speedMmPerSec > 0.0)
		{
			ins->setSpeed(tp.speedMmPerSec);
		}
		memberIds.push_back(ins->id());
		newMotion.push_back(std::move(ins));
	}
	if (memberIds.empty())
	{
		if (errMsg)
		{
			*errMsg = "no reachable points";
		}
		return false;
	}
	for (std::shared_ptr<Base>& motion : newMotion)
	{
		program.steps.push_back(std::move(motion));
	}
	if (program.steps.empty())
	{
		if (errMsg)
		{
			*errMsg = "no reachable points";
		}
		return false;
	}
	InstructionGroup group;
	group.id = makeGroupId();
	const std::string featureId = rawTrajectoryFeatureId(trajectory);
	group.name = featureId.empty() ? "RawTrajectory" : featureId;
	group.memberInstructionIds = std::move(memberIds);
	if (pathPlanInstructionId && !pathPlanInstructionId->empty())
	{
		group.role = InstructionGroupRole::PathPlanOutput;
		group.pathPlanInstructionId = *pathPlanInstructionId;
	}
	program.groups.push_back(std::move(group));
	if (outGroupId)
	{
		*outGroupId = program.groups.back().id;
	}
	return true;
}

std::string rawTrajectoryToPreviewPolylineXyz(const RawTrajectory& trajectory)
{
	nlohmann::json arr = nlohmann::json::array();
	for (const TrajectoryPoint& tp : trajectory.points)
	{
		arr.push_back(tp.poseMm.x);
		arr.push_back(tp.poseMm.y);
		arr.push_back(tp.poseMm.z);
	}
	return arr.dump();
}

std::string rawTrajectoryWorkpieceBackendId(const RawTrajectory& trajectory)
{
	if (trajectory.sourceFeatureJson.empty())
	{
		return {};
	}
	geoalgo::FeatureSpec spec{};
	std::string err;
	if (!geometry_backend_ops::featureSpecFromJson(trajectory.sourceFeatureJson, spec, &err))
	{
		return {};
	}
	return spec.workpiece.backendIdUtf8;
}

std::string rawTrajectoryFeatureId(const RawTrajectory& trajectory)
{
	if (trajectory.sourceFeatureJson.empty())
	{
		return {};
	}
	geoalgo::FeatureSpec spec{};
	std::string err;
	if (!geometry_backend_ops::featureSpecFromJson(trajectory.sourceFeatureJson, spec, &err))
	{
		return {};
	}
	return spec.featureId;
}

std::string rawTrajectoryReachabilityColorsJson(const RawTrajectory& trajectory)
{
	nlohmann::json arr = nlohmann::json::array();
	for (const TrajectoryPoint& tp : trajectory.points)
	{
		arr.push_back(tp.reachable ? "#00ff00" : "#ff0000");
	}
	return arr.dump();
}

} // namespace RobotInstruction
