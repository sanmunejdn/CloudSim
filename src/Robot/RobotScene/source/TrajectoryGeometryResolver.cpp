/// @file TrajectoryGeometryResolver.cpp
/// @brief TrajectoryGeometryResolver 实现

#include "TrajectoryGeometryResolver.h"

#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "PointCloudBackendOps.h"
#include "RobotSceneGeometryProjection.h"
#include "RobotSceneNonRigidTrajectoryWarp.h"
#include "ShapeQuery.h"
#include "TrajectoryProjection.h"

#include <cmath>
#include <limits>
#include <unordered_map>

#include <RigidTransform.h>
#include <TrajectoryUnifiedScope.h>

namespace RobotInstruction
{
namespace
{
TrajectoryGeometryResolveFn g_geometryResolver;
std::unordered_map<std::string, TrajectoryGeometrySnapshot> g_geometryCache;
std::size_t g_lastProjectionMissCount = 0;

void transformPointModelToWorld(const double modelToWorldColMajor16[16], const double modelMm[3], double worldMm[3])
{
	worldMm[0] = modelToWorldColMajor16[0] * modelMm[0] + modelToWorldColMajor16[4] * modelMm[1] +
				 modelToWorldColMajor16[8] * modelMm[2] + modelToWorldColMajor16[12];
	worldMm[1] = modelToWorldColMajor16[1] * modelMm[0] + modelToWorldColMajor16[5] * modelMm[1] +
				 modelToWorldColMajor16[9] * modelMm[2] + modelToWorldColMajor16[13];
	worldMm[2] = modelToWorldColMajor16[2] * modelMm[0] + modelToWorldColMajor16[6] * modelMm[1] +
				 modelToWorldColMajor16[10] * modelMm[2] + modelToWorldColMajor16[14];
}

void transformDirModelToWorld(const double modelToWorldColMajor16[16], const double modelDir[3], double worldDir[3])
{
	worldDir[0] = modelToWorldColMajor16[0] * modelDir[0] + modelToWorldColMajor16[4] * modelDir[1] +
				  modelToWorldColMajor16[8] * modelDir[2];
	worldDir[1] = modelToWorldColMajor16[1] * modelDir[0] + modelToWorldColMajor16[5] * modelDir[1] +
				  modelToWorldColMajor16[9] * modelDir[2];
	worldDir[2] = modelToWorldColMajor16[2] * modelDir[0] + modelToWorldColMajor16[6] * modelDir[1] +
				  modelToWorldColMajor16[10] * modelDir[2];
	const double len = std::sqrt(worldDir[0] * worldDir[0] + worldDir[1] * worldDir[1] + worldDir[2] * worldDir[2]);
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

void transformPointWorldToModel(const double worldToModelColMajor16[16], const double worldMm[3], double modelMm[3])
{
	modelMm[0] = worldToModelColMajor16[0] * worldMm[0] + worldToModelColMajor16[4] * worldMm[1] +
				 worldToModelColMajor16[8] * worldMm[2] + worldToModelColMajor16[12];
	modelMm[1] = worldToModelColMajor16[1] * worldMm[0] + worldToModelColMajor16[5] * worldMm[1] +
				 worldToModelColMajor16[9] * worldMm[2] + worldToModelColMajor16[13];
	modelMm[2] = worldToModelColMajor16[2] * worldMm[0] + worldToModelColMajor16[6] * worldMm[1] +
				 worldToModelColMajor16[10] * worldMm[2] + worldToModelColMajor16[14];
}

void transformDirWorldToModel(const double worldToModelColMajor16[16], const double worldDir[3], double modelDir[3])
{
	modelDir[0] = worldToModelColMajor16[0] * worldDir[0] + worldToModelColMajor16[4] * worldDir[1] +
				  worldToModelColMajor16[8] * worldDir[2];
	modelDir[1] = worldToModelColMajor16[1] * worldDir[0] + worldToModelColMajor16[5] * worldDir[1] +
				  worldToModelColMajor16[9] * worldDir[2];
	modelDir[2] = worldToModelColMajor16[2] * worldDir[0] + worldToModelColMajor16[6] * worldDir[1] +
				  worldToModelColMajor16[10] * worldDir[2];
	const double len = std::sqrt(modelDir[0] * modelDir[0] + modelDir[1] * modelDir[1] + modelDir[2] * modelDir[2]);
	if (len > 1e-9)
	{
		modelDir[0] /= len;
		modelDir[1] /= len;
		modelDir[2] /= len;
	}
}

Eigen::Vector3d resolveProjectDirection(const UnifiedTrajectoryPoint& point, const ProjectToGeometryParams& params)
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
			point.poseMm.x, point.poseMm.y, point.poseMm.z, point.eulerDeg.x, point.eulerDeg.y, point.eulerDeg.z);
		dir = tf.rotation().toRotationMatrix() * dir;
		if (dir.norm() > 1e-9)
		{
			dir.normalize();
		}
	}
	return dir;
}

bool projectPointOntoSnapshot(const TrajectoryGeometrySnapshot& snap, const double originWorldMm[3],
							  const double dirWorldUnit[3], const double maxDistanceMm, const double hitRadiusMm,
							  double outHitWorldMm[3], bool& outHit, std::string* errMsg)
{
	outHit = false;
	switch (snap.kind)
	{
	case TrajectoryGeometryKind::TriangleMesh:
		return geoalgo::projectRayOntoTriangleSoup(originWorldMm, dirWorldUnit, maxDistanceMm, snap.triangleSoupWorldMm,
												   outHitWorldMm, outHit);
	case TrajectoryGeometryKind::PointCloud:
		return geoalgo::projectRayOntoPointCloud(originWorldMm, dirWorldUnit, maxDistanceMm, hitRadiusMm,
												 snap.positionsWorldMm, outHitWorldMm, outHit);
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
		geoalgo::Point3d o{originModel[0], originModel[1], originModel[2]};
		geoalgo::Point3d d{dirModel[0], dirModel[1], dirModel[2]};
		if (!geoalgo::pickShapeFaceByModelRay(snap.brepShape, o, d, pick, errMsg))
		{
			return false;
		}
		if (!pick.hit)
		{
			return true;
		}
		const double hitModel[3] = {pick.hitPointModelMm.x, pick.hitPointModelMm.y, pick.hitPointModelMm.z};
		transformPointModelToWorld(snap.modelToWorldColMajor16, hitModel, outHitWorldMm);
		const double dist = std::sqrt((outHitWorldMm[0] - originWorldMm[0]) * (outHitWorldMm[0] - originWorldMm[0]) +
									  (outHitWorldMm[1] - originWorldMm[1]) * (outHitWorldMm[1] - originWorldMm[1]) +
									  (outHitWorldMm[2] - originWorldMm[2]) * (outHitWorldMm[2] - originWorldMm[2]));
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

bool resolveTrajectoryGeometry(const std::string& backendId, TrajectoryGeometrySnapshot& out, std::string* errMsg)
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

bool projectUnifiedToGeometry(UnifiedTrajectory& traj, const ProjectToGeometryParams& params, const OpScope& scope,
							  const RobotProgram* program, std::size_t* outMissCount, std::string* errMsg)
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
	const double dirLen = std::sqrt(params.directionX * params.directionX + params.directionY * params.directionY +
									params.directionZ * params.directionZ);
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
	const std::vector<std::size_t> indices = trajectory_algo::resolveScopedPointIndices(traj, scope, program);
	for (const std::size_t idx : indices)
	{
		if (idx >= traj.points.size())
		{
			continue;
		}
		UnifiedTrajectoryPoint& point = traj.points[idx];
		const Eigen::Vector3d dir = resolveProjectDirection(point, params);
		const double origin[3] = {point.poseMm.x, point.poseMm.y, point.poseMm.z};
		const double dirArr[3] = {dir.x(), dir.y(), dir.z()};
		double hit[3]{};
		bool hitOk = false;
		if (!projectPointOntoSnapshot(snap, origin, dirArr, params.maxDistanceMm, params.pointCloudHitRadiusMm, hit,
									  hitOk, errMsg))
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

bool RobotSceneGeometryProjection::project(UnifiedTrajectory& traj, const ProjectToGeometryParams& params,
										   const OpScope& scope, const RobotProgram* program, std::size_t* missCount,
										   std::string* errMsg) const
{
	return projectUnifiedToGeometry(traj, params, scope, program, missCount, errMsg);
}

const RobotSceneGeometryProjection& robotSceneGeometryProjection()
{
	static const RobotSceneGeometryProjection instance{};
	return instance;
}

namespace
{
struct MeshTriangleBinding
{
	int faceIndex = -1;
	float w0 = 0.0f;
	float w1 = 0.0f;
	float w2 = 0.0f;
	double bindDistance = 0.0;
	bool valid = false;
};

struct PointCloudBinding
{
	std::size_t pointIndex = 0;
	double bindDistance = 0.0;
	bool valid = false;
};

struct TrajectoryPointBinding
{
	std::size_t trajIndex = 0;
	bool isMesh = false;
	MeshTriangleBinding meshBinding;
	PointCloudBinding pointBinding;
};

double closestPointOnTriangleForBind(const double p[3], const double a[3], const double b[3], const double c[3],
									 double closest[3], double& w0, double& w1, double& w2)
{
	const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
	const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
	const double ap[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
	const double d1 = ab[0] * ap[0] + ab[1] * ap[1] + ab[2] * ap[2];
	const double d2 = ac[0] * ap[0] + ac[1] * ap[1] + ac[2] * ap[2];
	if (d1 <= 0.0 && d2 <= 0.0)
	{
		closest[0] = a[0];
		closest[1] = a[1];
		closest[2] = a[2];
		w0 = 1.0;
		w1 = 0.0;
		w2 = 0.0;
		const double dx = p[0] - a[0];
		const double dy = p[1] - a[1];
		const double dz = p[2] - a[2];
		return dx * dx + dy * dy + dz * dz;
	}
	const double bp[3] = {p[0] - b[0], p[1] - b[1], p[2] - b[2]};
	const double d3 = ab[0] * bp[0] + ab[1] * bp[1] + ab[2] * bp[2];
	const double d4 = ac[0] * bp[0] + ac[1] * bp[1] + ac[2] * bp[2];
	if (d3 >= 0.0 && d4 <= d3)
	{
		closest[0] = b[0];
		closest[1] = b[1];
		closest[2] = b[2];
		w0 = 0.0;
		w1 = 1.0;
		w2 = 0.0;
		const double dx = p[0] - b[0];
		const double dy = p[1] - b[1];
		const double dz = p[2] - b[2];
		return dx * dx + dy * dy + dz * dz;
	}
	const double vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
	{
		const double v = d1 / (d1 - d3);
		closest[0] = a[0] + v * ab[0];
		closest[1] = a[1] + v * ab[1];
		closest[2] = a[2] + v * ab[2];
		w0 = 1.0 - v;
		w1 = v;
		w2 = 0.0;
		const double dx = p[0] - closest[0];
		const double dy = p[1] - closest[1];
		const double dz = p[2] - closest[2];
		return dx * dx + dy * dy + dz * dz;
	}
	const double cp[3] = {p[0] - c[0], p[1] - c[1], p[2] - c[2]};
	const double d5 = ab[0] * cp[0] + ab[1] * cp[1] + ab[2] * cp[2];
	const double d6 = ac[0] * cp[0] + ac[1] * cp[1] + ac[2] * cp[2];
	if (d6 >= 0.0 && d5 <= d6)
	{
		closest[0] = c[0];
		closest[1] = c[1];
		closest[2] = c[2];
		w0 = 0.0;
		w1 = 0.0;
		w2 = 1.0;
		const double dx = p[0] - c[0];
		const double dy = p[1] - c[1];
		const double dz = p[2] - c[2];
		return dx * dx + dy * dy + dz * dz;
	}
	const double vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
	{
		const double w = d2 / (d2 - d6);
		closest[0] = a[0] + w * ac[0];
		closest[1] = a[1] + w * ac[1];
		closest[2] = a[2] + w * ac[2];
		w0 = 1.0 - w;
		w1 = 0.0;
		w2 = w;
		const double dx = p[0] - closest[0];
		const double dy = p[1] - closest[1];
		const double dz = p[2] - closest[2];
		return dx * dx + dy * dy + dz * dz;
	}
	const double va = d3 * d6 - d5 * d4;
	if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
	{
		const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest[0] = b[0] + w * (c[0] - b[0]);
		closest[1] = b[1] + w * (c[1] - b[1]);
		closest[2] = b[2] + w * (c[2] - b[2]);
		w0 = 0.0;
		w1 = 1.0 - w;
		w2 = w;
		const double dx = p[0] - closest[0];
		const double dy = p[1] - closest[1];
		const double dz = p[2] - closest[2];
		return dx * dx + dy * dy + dz * dz;
	}
	const double denom = 1.0 / (va + vb + vc);
	const double v = vb * denom;
	const double w = vc * denom;
	closest[0] = a[0] + ab[0] * v + ac[0] * w;
	closest[1] = a[1] + ab[1] * v + ac[1] * w;
	closest[2] = a[2] + ab[2] * v + ac[2] * w;
	w0 = 1.0 - v - w;
	w1 = v;
	w2 = w;
	const double dx = p[0] - closest[0];
	const double dy = p[1] - closest[1];
	const double dz = p[2] - closest[2];
	return dx * dx + dy * dy + dz * dz;
}

MeshTriangleBinding bindPointToMeshSoup(const double p[3], const std::vector<float>& soup, const double maxBindDistance)
{
	MeshTriangleBinding best;
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		return best;
	}
	double bestDistSq = std::numeric_limits<double>::max();
	const int numFaces = static_cast<int>(soup.size() / 9U);
	for (int f = 0; f < numFaces; ++f)
	{
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		const double a[3] = {static_cast<double>(soup[base + 0U]), static_cast<double>(soup[base + 1U]),
							 static_cast<double>(soup[base + 2U])};
		const double b[3] = {static_cast<double>(soup[base + 3U]), static_cast<double>(soup[base + 4U]),
							 static_cast<double>(soup[base + 5U])};
		const double c[3] = {static_cast<double>(soup[base + 6U]), static_cast<double>(soup[base + 7U]),
							 static_cast<double>(soup[base + 8U])};
		double closest[3]{};
		double w0 = 0.0;
		double w1 = 0.0;
		double w2 = 0.0;
		const double distSq = closestPointOnTriangleForBind(p, a, b, c, closest, w0, w1, w2);
		if (distSq < bestDistSq)
		{
			bestDistSq = distSq;
			best.faceIndex = f;
			best.w0 = static_cast<float>(w0);
			best.w1 = static_cast<float>(w1);
			best.w2 = static_cast<float>(w2);
		}
	}
	const double sumW = static_cast<double>(best.w0 + best.w1 + best.w2);
	if (sumW > 1e-12)
	{
		best.w0 = static_cast<float>(static_cast<double>(best.w0) / sumW);
		best.w1 = static_cast<float>(static_cast<double>(best.w1) / sumW);
		best.w2 = static_cast<float>(static_cast<double>(best.w2) / sumW);
	}
	best.bindDistance = std::sqrt(bestDistSq);
	best.valid = best.bindDistance <= maxBindDistance;
	return best;
}

PointCloudBinding bindPointToPointCloud(const double p[3], const std::vector<float>& xyz, const double maxBindDistance)
{
	PointCloudBinding best;
	if (xyz.size() < 3U)
	{
		return best;
	}
	double bestDistSq = std::numeric_limits<double>::max();
	const std::size_t pointCount = xyz.size() / 3U;
	for (std::size_t i = 0; i < pointCount; ++i)
	{
		const std::size_t b = i * 3U;
		const double dx = p[0] - static_cast<double>(xyz[b]);
		const double dy = p[1] - static_cast<double>(xyz[b + 1U]);
		const double dz = p[2] - static_cast<double>(xyz[b + 2U]);
		const double distSq = dx * dx + dy * dy + dz * dz;
		if (distSq < bestDistSq)
		{
			bestDistSq = distSq;
			best.pointIndex = i;
		}
	}
	best.bindDistance = std::sqrt(bestDistSq);
	best.valid = best.bindDistance <= maxBindDistance;
	return best;
}

bool isSupportedNonRigidGeometryKind(const TrajectoryGeometryKind kind)
{
	return kind == TrajectoryGeometryKind::PointCloud || kind == TrajectoryGeometryKind::TriangleMesh;
}

std::size_t geometryPrimitiveCount(const TrajectoryGeometrySnapshot& snap)
{
	if (snap.kind == TrajectoryGeometryKind::TriangleMesh)
	{
		return snap.triangleSoupWorldMm.size() / 9U;
	}
	if (snap.kind == TrajectoryGeometryKind::PointCloud)
	{
		return snap.positionsWorldMm.size() / 3U;
	}
	return 0;
}

point_cloud_backend_ops::PointCloudSpareParams toSpareParams(const NonRigidRegistrationParams& params)
{
	point_cloud_backend_ops::PointCloudSpareParams out;
	out.sampleRadiusRatio = params.sampleRadiusRatio;
	out.maxOuterIters = params.maxOuterIters;
	out.rigidPreAlign = params.rigidPreAlign;
	out.voxelPrefilterMm = params.voxelPrefilterMm;
	return out;
}

void fillPointCloudFromWorldXyz(PointCloudBackendData& pc, const std::vector<float>& xyz)
{
	pc.setPointBuffers(xyz, {});
}

void fillMeshFromWorldSoup(MeshBackendData& mesh, const std::vector<float>& soup)
{
	mesh.setTriangleSoup(soup);
}

struct NonRigidSpareCacheKey
{
	std::string srcId;
	std::string tgtId;
	TrajectoryGeometryKind srcKind = TrajectoryGeometryKind::PointCloud;
	TrajectoryGeometryKind tgtKind = TrajectoryGeometryKind::PointCloud;
	double maxBindDistanceMm = 0.0;
	double sampleRadiusRatio = 0.0;
	int maxOuterIters = 0;
	bool rigidPreAlign = false;
	double voxelPrefilterMm = 0.0;
};

struct NonRigidSpareCache
{
	NonRigidSpareCacheKey key{};
	std::vector<float> deformedMeshSoup;
	std::vector<float> deformedPointCloud;
	bool valid = false;
};

NonRigidSpareCache& nonRigidSpareCache()
{
	static NonRigidSpareCache cache;
	return cache;
}

bool cacheKeyEqual(const NonRigidSpareCacheKey& a, const NonRigidSpareCacheKey& b)
{
	return a.srcId == b.srcId && a.tgtId == b.tgtId && a.srcKind == b.srcKind && a.tgtKind == b.tgtKind &&
		   a.maxBindDistanceMm == b.maxBindDistanceMm && a.sampleRadiusRatio == b.sampleRadiusRatio &&
		   a.maxOuterIters == b.maxOuterIters && a.rigidPreAlign == b.rigidPreAlign &&
		   a.voxelPrefilterMm == b.voxelPrefilterMm;
}

NonRigidSpareCacheKey makeSpareCacheKey(const TrajectoryGeometrySnapshot& srcSnap,
										const TrajectoryGeometrySnapshot& tgtSnap,
										const NonRigidRegistrationParams& params)
{
	NonRigidSpareCacheKey key{};
	key.srcId = params.sourceBackendId;
	key.tgtId = params.targetBackendId;
	key.srcKind = srcSnap.kind;
	key.tgtKind = tgtSnap.kind;
	key.maxBindDistanceMm = params.maxBindDistanceMm;
	key.sampleRadiusRatio = params.sampleRadiusRatio;
	key.maxOuterIters = params.maxOuterIters;
	key.rigidPreAlign = params.rigidPreAlign;
	key.voxelPrefilterMm = params.voxelPrefilterMm;
	return key;
}

// 大网格未设采样比时加大半径，减少变形节点
NonRigidRegistrationParams withEffectiveSpareParams(const TrajectoryGeometrySnapshot& srcSnap,
													const NonRigidRegistrationParams& params)
{
	NonRigidRegistrationParams effective = params;
	if (effective.sampleRadiusRatio <= 0.0 && srcSnap.kind == TrajectoryGeometryKind::TriangleMesh &&
		geometryPrimitiveCount(srcSnap) > 20000U)
	{
		effective.sampleRadiusRatio = 5.0;
	}
	return effective;
}

bool runSpareRegistration(const TrajectoryGeometrySnapshot& srcSnap, const TrajectoryGeometrySnapshot& tgtSnap,
						  const NonRigidRegistrationParams& params, std::vector<float>& deformedMeshSoupOut,
						  std::vector<float>& deformedPointCloudOut, std::string* errMsg)
{
	const NonRigidRegistrationParams effective = withEffectiveSpareParams(srcSnap, params);
	const NonRigidSpareCacheKey key = makeSpareCacheKey(srcSnap, tgtSnap, effective);
	NonRigidSpareCache& cache = nonRigidSpareCache();
	if (cache.valid && cacheKeyEqual(cache.key, key))
	{
		deformedMeshSoupOut = cache.deformedMeshSoup;
		deformedPointCloudOut = cache.deformedPointCloud;
		return true;
	}

	const point_cloud_backend_ops::PointCloudSpareParams spareParams = toSpareParams(effective);
	point_cloud_backend_ops::PointCloudSpareResult spareResult;

	if (srcSnap.kind == TrajectoryGeometryKind::TriangleMesh && tgtSnap.kind == TrajectoryGeometryKind::TriangleMesh)
	{
		MeshBackendData srcMesh;
		fillMeshFromWorldSoup(srcMesh, srcSnap.triangleSoupWorldMm);
		MeshBackendData tgtMesh;
		fillMeshFromWorldSoup(tgtMesh, tgtSnap.triangleSoupWorldMm);
		if (!point_cloud_backend_ops::nonRigidRegisterMeshSpare(srcMesh, nullptr, &tgtMesh, spareResult, spareParams,
																errMsg))
		{
			return false;
		}
		deformedMeshSoupOut = srcMesh.triangleSoup();
	}
	else if (srcSnap.kind == TrajectoryGeometryKind::TriangleMesh && tgtSnap.kind == TrajectoryGeometryKind::PointCloud)
	{
		MeshBackendData srcMesh;
		fillMeshFromWorldSoup(srcMesh, srcSnap.triangleSoupWorldMm);
		PointCloudBackendData tgtPc;
		fillPointCloudFromWorldXyz(tgtPc, tgtSnap.positionsWorldMm);
		if (!point_cloud_backend_ops::nonRigidRegisterMeshSpare(srcMesh, &tgtPc, nullptr, spareResult, spareParams,
																errMsg))
		{
			return false;
		}
		deformedMeshSoupOut = srcMesh.triangleSoup();
	}
	else if (srcSnap.kind == TrajectoryGeometryKind::PointCloud && tgtSnap.kind == TrajectoryGeometryKind::TriangleMesh)
	{
		PointCloudBackendData srcPc;
		fillPointCloudFromWorldXyz(srcPc, srcSnap.positionsWorldMm);
		MeshBackendData tgtMesh;
		fillMeshFromWorldSoup(tgtMesh, tgtSnap.triangleSoupWorldMm);
		if (!point_cloud_backend_ops::nonRigidRegisterPointCloudToMeshSpare(srcPc, tgtMesh, spareResult, spareParams,
																			errMsg))
		{
			return false;
		}
		deformedPointCloudOut = srcPc.pointPositionsXyz();
	}
	else if (srcSnap.kind == TrajectoryGeometryKind::PointCloud && tgtSnap.kind == TrajectoryGeometryKind::PointCloud)
	{
		PointCloudBackendData srcPc;
		fillPointCloudFromWorldXyz(srcPc, srcSnap.positionsWorldMm);
		PointCloudBackendData tgtPc;
		fillPointCloudFromWorldXyz(tgtPc, tgtSnap.positionsWorldMm);
		if (!point_cloud_backend_ops::nonRigidRegisterPointCloudsSpare(srcPc, tgtPc, spareResult, spareParams, errMsg))
		{
			return false;
		}
		deformedPointCloudOut = srcPc.pointPositionsXyz();
	}
	else
	{
		if (errMsg)
		{
			*errMsg = "unsupported source/target geometry combination";
		}
		return false;
	}

	cache.key = key;
	cache.deformedMeshSoup = deformedMeshSoupOut;
	cache.deformedPointCloud = deformedPointCloudOut;
	cache.valid = true;
	return true;
}

void applyMeshBinding(UnifiedTrajectoryPoint& point, const MeshTriangleBinding& binding,
					  const std::vector<float>& deformedSoup)
{
	if (!binding.valid || binding.faceIndex < 0)
	{
		return;
	}
	const std::size_t base = static_cast<std::size_t>(binding.faceIndex) * 9U;
	if (base + 8U >= deformedSoup.size())
	{
		return;
	}
	const double v0[3] = {static_cast<double>(deformedSoup[base + 0U]), static_cast<double>(deformedSoup[base + 1U]),
						  static_cast<double>(deformedSoup[base + 2U])};
	const double v1[3] = {static_cast<double>(deformedSoup[base + 3U]), static_cast<double>(deformedSoup[base + 4U]),
						  static_cast<double>(deformedSoup[base + 5U])};
	const double v2[3] = {static_cast<double>(deformedSoup[base + 6U]), static_cast<double>(deformedSoup[base + 7U]),
						  static_cast<double>(deformedSoup[base + 8U])};
	point.poseMm.x = static_cast<float>(binding.w0 * v0[0] + binding.w1 * v1[0] + binding.w2 * v2[0]);
	point.poseMm.y = static_cast<float>(binding.w0 * v0[1] + binding.w1 * v1[1] + binding.w2 * v2[1]);
	point.poseMm.z = static_cast<float>(binding.w0 * v0[2] + binding.w1 * v1[2] + binding.w2 * v2[2]);
}

void applyPointCloudBinding(UnifiedTrajectoryPoint& point, const PointCloudBinding& binding,
							const std::vector<float>& deformedXyz)
{
	if (!binding.valid)
	{
		return;
	}
	const std::size_t base = binding.pointIndex * 3U;
	if (base + 2U >= deformedXyz.size())
	{
		return;
	}
	point.poseMm.x = deformedXyz[base];
	point.poseMm.y = deformedXyz[base + 1U];
	point.poseMm.z = deformedXyz[base + 2U];
}

} // namespace

bool nonRigidWarpUnifiedTrajectory(UnifiedTrajectory& traj, const NonRigidRegistrationParams& params,
								   const OpScope& scope, const RobotProgram* program, std::size_t* outMissCount,
								   std::string* errMsg)
{
	if (outMissCount)
	{
		*outMissCount = 0;
	}
	if (traj.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "trajectory has no points";
		}
		return false;
	}
	if (params.sourceBackendId.empty() || params.targetBackendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "source or target backend is empty";
		}
		return false;
	}
	if (params.maxBindDistanceMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "max bind distance must be > 0";
		}
		return false;
	}

	TrajectoryGeometrySnapshot srcSnap{};
	if (!resolveTrajectoryGeometry(params.sourceBackendId, srcSnap, errMsg))
	{
		return false;
	}
	if (!isSupportedNonRigidGeometryKind(srcSnap.kind))
	{
		if (errMsg)
		{
			*errMsg = "source backend must be point cloud or mesh";
		}
		return false;
	}

	TrajectoryGeometrySnapshot tgtSnap{};
	if (!resolveTrajectoryGeometry(params.targetBackendId, tgtSnap, errMsg))
	{
		return false;
	}
	if (!isSupportedNonRigidGeometryKind(tgtSnap.kind))
	{
		if (errMsg)
		{
			*errMsg = "target backend must be point cloud or mesh";
		}
		return false;
	}

	const std::vector<std::size_t> indices = trajectory_algo::resolveScopedPointIndices(traj, scope, program);
	if (indices.empty())
	{
		if (errMsg)
		{
			*errMsg = "scope contains no trajectory points";
		}
		return false;
	}

	const bool sourceIsMesh = srcSnap.kind == TrajectoryGeometryKind::TriangleMesh;
	std::vector<TrajectoryPointBinding> bindings;
	bindings.reserve(indices.size());
	for (const std::size_t idx : indices)
	{
		if (idx >= traj.points.size())
		{
			continue;
		}
		TrajectoryPointBinding item{};
		item.trajIndex = idx;
		item.isMesh = sourceIsMesh;
		const double p[3] = {static_cast<double>(traj.points[idx].poseMm.x),
							 static_cast<double>(traj.points[idx].poseMm.y),
							 static_cast<double>(traj.points[idx].poseMm.z)};
		if (sourceIsMesh)
		{
			item.meshBinding = bindPointToMeshSoup(p, srcSnap.triangleSoupWorldMm, params.maxBindDistanceMm);
		}
		else
		{
			item.pointBinding = bindPointToPointCloud(p, srcSnap.positionsWorldMm, params.maxBindDistanceMm);
		}
		bindings.push_back(item);
	}
	if (bindings.empty())
	{
		if (errMsg)
		{
			*errMsg = "no trajectory points in scope";
		}
		return false;
	}

	std::vector<float> deformedMeshSoup;
	std::vector<float> deformedPointCloud;
	if (!runSpareRegistration(srcSnap, tgtSnap, params, deformedMeshSoup, deformedPointCloud, errMsg))
	{
		return false;
	}

	std::size_t missCount = 0;
	for (const TrajectoryPointBinding& item : bindings)
	{
		if (item.trajIndex >= traj.points.size())
		{
			continue;
		}
		UnifiedTrajectoryPoint& point = traj.points[item.trajIndex];
		if (item.isMesh)
		{
			if (!item.meshBinding.valid)
			{
				++missCount;
				continue;
			}
			applyMeshBinding(point, item.meshBinding, deformedMeshSoup);
		}
		else
		{
			if (!item.pointBinding.valid)
			{
				++missCount;
				continue;
			}
			applyPointCloudBinding(point, item.pointBinding, deformedPointCloud);
		}
	}
	if (outMissCount)
	{
		*outMissCount = missCount;
	}
	return true;
}

bool RobotSceneNonRigidTrajectoryWarp::warp(UnifiedTrajectory& traj, const NonRigidRegistrationParams& params,
											const OpScope& scope, const RobotProgram* program, std::size_t* missCount,
											std::string* errMsg) const
{
	return nonRigidWarpUnifiedTrajectory(traj, params, scope, program, missCount, errMsg);
}

const RobotSceneNonRigidTrajectoryWarp& robotSceneNonRigidTrajectoryWarp()
{
	static const RobotSceneNonRigidTrajectoryWarp instance{};
	return instance;
}

} // namespace RobotInstruction
