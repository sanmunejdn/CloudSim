#include "TrajectoryGeometryResolver.h"

#include "RobotSceneGeometryProjection.h"
#include "ShapeQuery.h"
#include "TrajectoryProjection.h"

#include <TrajectoryUnifiedScope.h>

#include <RigidTransform.h>

#include <cmath>
#include <unordered_map>

namespace RobotInstruction
{
namespace
{

TrajectoryGeometryResolveFn g_geometryResolver;
std::unordered_map<std::string, TrajectoryGeometrySnapshot> g_geometryCache;
std::size_t g_lastProjectionMissCount = 0;

void transformPointModelToWorld(
	const double modelToWorldColMajor16[16],
	const double modelMm[3],
	double worldMm[3])
{
	worldMm[0] = modelToWorldColMajor16[0] * modelMm[0]
		+ modelToWorldColMajor16[4] * modelMm[1]
		+ modelToWorldColMajor16[8] * modelMm[2]
		+ modelToWorldColMajor16[12];
	worldMm[1] = modelToWorldColMajor16[1] * modelMm[0]
		+ modelToWorldColMajor16[5] * modelMm[1]
		+ modelToWorldColMajor16[9] * modelMm[2]
		+ modelToWorldColMajor16[13];
	worldMm[2] = modelToWorldColMajor16[2] * modelMm[0]
		+ modelToWorldColMajor16[6] * modelMm[1]
		+ modelToWorldColMajor16[10] * modelMm[2]
		+ modelToWorldColMajor16[14];
}

void transformDirModelToWorld(
	const double modelToWorldColMajor16[16],
	const double modelDir[3],
	double worldDir[3])
{
	worldDir[0] = modelToWorldColMajor16[0] * modelDir[0]
		+ modelToWorldColMajor16[4] * modelDir[1]
		+ modelToWorldColMajor16[8] * modelDir[2];
	worldDir[1] = modelToWorldColMajor16[1] * modelDir[0]
		+ modelToWorldColMajor16[5] * modelDir[1]
		+ modelToWorldColMajor16[9] * modelDir[2];
	worldDir[2] = modelToWorldColMajor16[2] * modelDir[0]
		+ modelToWorldColMajor16[6] * modelDir[1]
		+ modelToWorldColMajor16[10] * modelDir[2];
	const double len = std::sqrt(
		worldDir[0] * worldDir[0] + worldDir[1] * worldDir[1] + worldDir[2] * worldDir[2]);
	if (len > 1e-9)
	{
		worldDir[0] /= len;
		worldDir[1] /= len;
		worldDir[2] /= len;
	}
}

bool invertColMajor4x4(const double m[16], double out[16])
{
	const double a = m[0];
	const double b = m[4];
	const double c = m[8];
	const double d = m[12];
	const double e = m[1];
	const double f = m[5];
	const double g = m[9];
	const double h = m[13];
	const double i = m[2];
	const double j = m[6];
	const double k = m[10];
	const double l = m[14];
	const double mm = m[3];
	const double n = m[7];
	const double o = m[11];
	const double p = m[15];
	out[0] = f * (k * p - l * o) - j * (g * p - h * o) + n * (g * l - h * k);
	out[4] = -(b * (k * p - l * o) - i * (g * p - h * o) + mm * (g * l - h * k));
	out[8] = b * (j * p - n * o) - e * (k * p - l * o) + mm * (j * l - n * k);
	out[12] = -(b * (j * o - n * k) - e * (g * o - h * k) + i * (g * n - h * j));
	out[1] = -(e * (k * p - l * o) - i * (f * p - h * o) + mm * (f * l - h * k));
	out[5] = a * (k * p - l * o) - c * (g * p - h * o) + d * (g * l - h * k);
	out[9] = -(a * (j * p - n * o) - c * (f * p - h * o) + d * (j * l - n * k));
	out[13] = a * (j * o - n * k) - c * (f * o - h * k) + d * (f * n - h * j);
	out[2] = e * (j * p - n * o) - i * (f * p - h * o) + mm * (f * n - h * j);
	out[6] = -(a * (j * p - n * o) - b * (i * p - l * o) + d * (i * n - l * j));
	out[10] = a * (f * p - h * o) - b * (e * p - h * o) + c * (e * n - h * mm);
	out[14] = -(a * (f * o - h * mm) - b * (e * o - h * i) + c * (e * mm - h * e));
	out[3] = -(e * (j * o - n * k) - i * (f * o - h * k) + mm * (f * k - h * j));
	out[7] = a * (j * o - n * k) - b * (i * o - l * k) + c * (i * n - l * mm);
	out[11] = -(a * (f * o - h * mm) - b * (e * o - h * i) + c * (e * mm - f * i));
	out[15] = a * (f * k - h * j) - b * (e * k - h * i) + c * (e * j - f * i);
	const double det = a * out[0] + e * out[4] + i * out[8] + mm * out[12];
	if (std::fabs(det) < 1e-12)
	{
		return false;
	}
	const double inv = 1.0 / det;
	for (int idx = 0; idx < 16; ++idx)
	{
		out[idx] *= inv;
	}
	return true;
}

void transformPointWorldToModel(
	const double worldToModelColMajor16[16],
	const double worldMm[3],
	double modelMm[3])
{
	modelMm[0] = worldToModelColMajor16[0] * worldMm[0]
		+ worldToModelColMajor16[4] * worldMm[1]
		+ worldToModelColMajor16[8] * worldMm[2]
		+ worldToModelColMajor16[12];
	modelMm[1] = worldToModelColMajor16[1] * worldMm[0]
		+ worldToModelColMajor16[5] * worldMm[1]
		+ worldToModelColMajor16[9] * worldMm[2]
		+ worldToModelColMajor16[13];
	modelMm[2] = worldToModelColMajor16[2] * worldMm[0]
		+ worldToModelColMajor16[6] * worldMm[1]
		+ worldToModelColMajor16[10] * worldMm[2]
		+ worldToModelColMajor16[14];
}

void transformDirWorldToModel(
	const double worldToModelColMajor16[16],
	const double worldDir[3],
	double modelDir[3])
{
	modelDir[0] = worldToModelColMajor16[0] * worldDir[0]
		+ worldToModelColMajor16[4] * worldDir[1]
		+ worldToModelColMajor16[8] * worldDir[2];
	modelDir[1] = worldToModelColMajor16[1] * worldDir[0]
		+ worldToModelColMajor16[5] * worldDir[1]
		+ worldToModelColMajor16[9] * worldDir[2];
	modelDir[2] = worldToModelColMajor16[2] * worldDir[0]
		+ worldToModelColMajor16[6] * worldDir[1]
		+ worldToModelColMajor16[10] * worldDir[2];
	const double len = std::sqrt(
		modelDir[0] * modelDir[0] + modelDir[1] * modelDir[1] + modelDir[2] * modelDir[2]);
	if (len > 1e-9)
	{
		modelDir[0] /= len;
		modelDir[1] /= len;
		modelDir[2] /= len;
	}
}

Eigen::Vector3d resolveProjectDirection(
	const UnifiedTrajectoryPoint& point,
	const ProjectToGeometryParams& params)
{
	Eigen::Vector3d dir(params.directionX, params.directionY, params.directionZ);
	if (dir.norm() < 1e-9)
	{
		dir = Eigen::Vector3d(0.0, 0.0, -1.0);
	}
	else
	{
		dir.normalize();
	}
	if (params.directionFrame == TransformReferenceFrame::Body)
	{
		const engine::RigidTransform tf = engine::RigidTransform::fromTranslationEulerDeg(
			point.poseMm.x,
			point.poseMm.y,
			point.poseMm.z,
			point.eulerDeg.x,
			point.eulerDeg.y,
			point.eulerDeg.z);
		dir = tf.rotation().toRotationMatrix() * dir;
		if (dir.norm() > 1e-9)
		{
			dir.normalize();
		}
	}
	return dir;
}

bool projectPointOntoSnapshot(
	const TrajectoryGeometrySnapshot& snap,
	const double originWorldMm[3],
	const double dirWorldUnit[3],
	const double maxDistanceMm,
	const double hitRadiusMm,
	double outHitWorldMm[3],
	bool& outHit,
	std::string* errMsg)
{
	outHit = false;
	switch (snap.kind)
	{
	case TrajectoryGeometryKind::TriangleMesh:
		return geoalgo::projectRayOntoTriangleSoup(
			originWorldMm,
			dirWorldUnit,
			maxDistanceMm,
			snap.triangleSoupWorldMm,
			outHitWorldMm,
			outHit);
	case TrajectoryGeometryKind::PointCloud:
		return geoalgo::projectRayOntoPointCloud(
			originWorldMm,
			dirWorldUnit,
			maxDistanceMm,
			hitRadiusMm,
			snap.positionsWorldMm,
			outHitWorldMm,
			outHit);
	case TrajectoryGeometryKind::Brep:
	{
		if (snap.brepShape.isNull() || !snap.hasModelToWorld)
		{
			if (errMsg)
			{
				*errMsg = "invalid brep geometry snapshot";
			}
			return false;
		}
		double worldToModel[16]{};
		if (!invertColMajor4x4(snap.modelToWorldColMajor16, worldToModel))
		{
			if (errMsg)
			{
				*errMsg = "failed to invert brep transform";
			}
			return false;
		}
		double originModel[3]{};
		double dirModel[3]{};
		transformPointWorldToModel(worldToModel, originWorldMm, originModel);
		transformDirWorldToModel(worldToModel, dirWorldUnit, dirModel);
		geoalgo::ShapeRayPickResult pick{};
		geoalgo::Point3d o{ originModel[0], originModel[1], originModel[2] };
		geoalgo::Point3d d{ dirModel[0], dirModel[1], dirModel[2] };
		if (!geoalgo::pickShapeFaceByModelRay(snap.brepShape, o, d, pick, errMsg))
		{
			return false;
		}
		if (!pick.hit)
		{
			return true;
		}
		const double hitModel[3] = { pick.hitPointModelMm.x, pick.hitPointModelMm.y, pick.hitPointModelMm.z };
		transformPointModelToWorld(snap.modelToWorldColMajor16, hitModel, outHitWorldMm);
		const double dist = std::sqrt(
			(outHitWorldMm[0] - originWorldMm[0]) * (outHitWorldMm[0] - originWorldMm[0])
			+ (outHitWorldMm[1] - originWorldMm[1]) * (outHitWorldMm[1] - originWorldMm[1])
			+ (outHitWorldMm[2] - originWorldMm[2]) * (outHitWorldMm[2] - originWorldMm[2]));
		if (dist <= maxDistanceMm)
		{
			outHit = true;
		}
		return true;
	}
	default:
		return true;
	}
}

} // namespace

void setTrajectoryGeometryResolver(TrajectoryGeometryResolveFn fn)
{
	g_geometryResolver = std::move(fn);
	g_geometryCache.clear();
}

void clearTrajectoryGeometryResolver()
{
	g_geometryResolver = nullptr;
	g_geometryCache.clear();
}

bool resolveTrajectoryGeometry(
	const std::string& backendId,
	TrajectoryGeometrySnapshot& out,
	std::string* errMsg)
{
	if (backendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty backend id";
		}
		return false;
	}
	const auto cached = g_geometryCache.find(backendId);
	if (cached != g_geometryCache.end())
	{
		out = cached->second;
		return true;
	}
	if (!g_geometryResolver)
	{
		if (errMsg)
		{
			*errMsg = "trajectory geometry resolver not registered";
		}
		return false;
	}
	if (!g_geometryResolver(backendId, out, errMsg))
	{
		return false;
	}
	g_geometryCache[backendId] = out;
	return true;
}

std::size_t trajectoryProjectionMissCount()
{
	return g_lastProjectionMissCount;
}

void resetTrajectoryProjectionMissCount()
{
	g_lastProjectionMissCount = 0;
}

bool projectUnifiedToGeometry(
	UnifiedTrajectory& traj,
	const ProjectToGeometryParams& params,
	const OpScope& scope,
	const RobotProgram* program,
	std::size_t* outMissCount,
	std::string* errMsg)
{
	g_lastProjectionMissCount = 0;
	if (outMissCount)
	{
		*outMissCount = 0;
	}
	if (traj.points.empty())
	{
		return false;
	}
	if (params.targetBackendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "project target backend is empty";
		}
		return false;
	}
	const double dirLen = std::sqrt(
		params.directionX * params.directionX
		+ params.directionY * params.directionY
		+ params.directionZ * params.directionZ);
	if (dirLen < 1e-6)
	{
		if (errMsg)
		{
			*errMsg = "project direction must be non-zero";
		}
		return false;
	}
	TrajectoryGeometrySnapshot snap{};
	if (!resolveTrajectoryGeometry(params.targetBackendId, snap, errMsg))
	{
		return false;
	}
	const std::vector<std::size_t> indices =
		trajectory_algo::resolveScopedPointIndices(traj, scope, program);
	for (const std::size_t idx : indices)
	{
		if (idx >= traj.points.size())
		{
			continue;
		}
		UnifiedTrajectoryPoint& point = traj.points[idx];
		const Eigen::Vector3d dir = resolveProjectDirection(point, params);
		const double origin[3] = { point.poseMm.x, point.poseMm.y, point.poseMm.z };
		const double dirArr[3] = { dir.x(), dir.y(), dir.z() };
		double hit[3]{};
		bool hitOk = false;
		if (!projectPointOntoSnapshot(
				snap,
				origin,
				dirArr,
				params.maxDistanceMm,
				params.pointCloudHitRadiusMm,
				hit,
				hitOk,
				errMsg))
		{
			return false;
		}
		if (!hitOk)
		{
			++g_lastProjectionMissCount;
			if (outMissCount)
			{
				++(*outMissCount);
			}
			continue;
		}
		point.poseMm.x = hit[0];
		point.poseMm.y = hit[1];
		point.poseMm.z = hit[2];
	}
	return true;
}

bool RobotSceneGeometryProjection::project(
	UnifiedTrajectory& traj,
	const ProjectToGeometryParams& params,
	const OpScope& scope,
	const RobotProgram* program,
	std::size_t* missCount,
	std::string* errMsg) const
{
	return projectUnifiedToGeometry(traj, params, scope, program, missCount, errMsg);
}

const RobotSceneGeometryProjection& robotSceneGeometryProjection()
{
	static const RobotSceneGeometryProjection instance{};
	return instance;
}

} // namespace RobotInstruction
