/// @file TrajectoryGeometryResolver.cpp
/// @brief TrajectoryGeometryResolver 实现

#include "TrajectoryGeometryResolver.h"

#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "PointCloudBackendOps.h"
#include "RobotSceneGeometryProjection.h"
#include "RobotSceneNonRigidTrajectoryWarp.h"
#include "RawTrajectory.h"
#include "RunLogger.h"
#include "ShapeQuery.h"
#include "TrajectoryProjection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <Adapters.h>
#include <RigidTransform.h>
#include <TrajectoryUnifiedScope.h>

namespace RobotInstruction
{
namespace
{
TrajectoryGeometryResolveFn g_geometryResolver;
std::size_t g_lastProjectionMissCount = 0;
NonRigidWarpLastStats g_lastNonRigidStats{};

void clearNonRigidSpareCache();

// modelToWorldColMajor16 与 BackendMat4/OSG 同序：须经 Adapters，不能当 Eigen 列向量 M*p（平移不在 v[12..14]）
engine::RigidTransform rigidFromModelToWorld16(const double modelToWorldColMajor16[16])
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = modelToWorldColMajor16[i];
	}
	return engine::rigidTransformFromColMajor(cm);
}

void transformPointModelToWorld(const double modelToWorldColMajor16[16], const double modelMm[3], double worldMm[3])
{
	const Eigen::Vector3d out =
		rigidFromModelToWorld16(modelToWorldColMajor16).isometry() *
		Eigen::Vector3d(modelMm[0], modelMm[1], modelMm[2]);
	worldMm[0] = out.x();
	worldMm[1] = out.y();
	worldMm[2] = out.z();
}

void transformDirModelToWorld(const double modelToWorldColMajor16[16], const double modelDir[3], double worldDir[3])
{
	const Eigen::Vector3d out =
		rigidFromModelToWorld16(modelToWorldColMajor16).isometry().linear() *
		Eigen::Vector3d(modelDir[0], modelDir[1], modelDir[2]);
	const double len = out.norm();
	if (len > 1e-9)
	{
		worldDir[0] = out.x() / len;
		worldDir[1] = out.y() / len;
		worldDir[2] = out.z() / len;
	}
	else
	{
		worldDir[0] = out.x();
		worldDir[1] = out.y();
		worldDir[2] = out.z();
	}
}

void transformPointWorldToModel(const double modelToWorldColMajor16[16], const double worldMm[3], double modelMm[3])
{
	const Eigen::Vector3d out =
		rigidFromModelToWorld16(modelToWorldColMajor16).inverse().isometry() *
		Eigen::Vector3d(worldMm[0], worldMm[1], worldMm[2]);
	modelMm[0] = out.x();
	modelMm[1] = out.y();
	modelMm[2] = out.z();
}

void transformDirWorldToModel(const double modelToWorldColMajor16[16], const double worldDir[3], double modelDir[3])
{
	const Eigen::Vector3d out =
		rigidFromModelToWorld16(modelToWorldColMajor16).inverse().isometry().linear() *
		Eigen::Vector3d(worldDir[0], worldDir[1], worldDir[2]);
	const double len = out.norm();
	if (len > 1e-9)
	{
		modelDir[0] = out.x() / len;
		modelDir[1] = out.y() / len;
		modelDir[2] = out.z() / len;
	}
	else
	{
		modelDir[0] = out.x();
		modelDir[1] = out.y();
		modelDir[2] = out.z();
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
		double originModel[3]{};
		double dirModel[3]{};
		transformPointWorldToModel(snap.modelToWorldColMajor16, originWorldMm, originModel);
		transformDirWorldToModel(snap.modelToWorldColMajor16, dirWorldUnit, dirModel);
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

void invalidateTrajectoryGeometryCache()
{
	clearNonRigidSpareCache();
}

void setTrajectoryGeometryResolver(TrajectoryGeometryResolveFn fn)
{
	g_geometryResolver = std::move(fn);
	invalidateTrajectoryGeometryCache();
}

void clearTrajectoryGeometryResolver()
{
	g_geometryResolver = nullptr;
	invalidateTrajectoryGeometryCache();
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
	// 场景物体位姿常变：禁止按 backendId 长缓存，每次从 Host 重烘焙世界坐标
	if (!g_geometryResolver)
	{
		if (errMsg)
		{
			*errMsg = "trajectory geometry resolver not registered";
		}
		return false;
	}
	return g_geometryResolver(backendId, out, errMsg);
}

std::size_t trajectoryProjectionMissCount()
{
	return g_lastProjectionMissCount;
}

void resetTrajectoryProjectionMissCount()
{
	g_lastProjectionMissCount = 0;
}

NonRigidWarpLastStats trajectoryNonRigidLastStats()
{
	return g_lastNonRigidStats;
}

void resetTrajectoryNonRigidLastStats()
{
	g_lastNonRigidStats = NonRigidWarpLastStats{};
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

// 世界坐标指纹：源/目标平移旋转后必须使 SPARE 缓存失效
struct GeomWorldStamp
{
	std::size_t floatCount = 0;
	float first[3]{};
	float last[3]{};
	double sum = 0.0;
};

struct NonRigidSpareCacheKey
{
	std::string srcId;
	std::string tgtId;
	TrajectoryGeometryKind srcKind = TrajectoryGeometryKind::PointCloud;
	TrajectoryGeometryKind tgtKind = TrajectoryGeometryKind::PointCloud;
	GeomWorldStamp srcStamp{};
	GeomWorldStamp tgtStamp{};
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
	double meanErrorMm = 0.0;
	int deformationNodeCount = 0;
	bool valid = false;
};

NonRigidSpareCache& nonRigidSpareCache()
{
	static NonRigidSpareCache cache;
	return cache;
}

void clearNonRigidSpareCache()
{
	NonRigidSpareCache& cache = nonRigidSpareCache();
	cache.valid = false;
	cache.deformedMeshSoup.clear();
	cache.deformedPointCloud.clear();
	cache.meanErrorMm = 0.0;
	cache.deformationNodeCount = 0;
	cache.key = NonRigidSpareCacheKey{};
}

GeomWorldStamp makeWorldStamp(const std::vector<float>& xyz)
{
	GeomWorldStamp stamp{};
	stamp.floatCount = xyz.size();
	if (xyz.size() >= 3U)
	{
		stamp.first[0] = xyz[0];
		stamp.first[1] = xyz[1];
		stamp.first[2] = xyz[2];
		const std::size_t n = xyz.size();
		stamp.last[0] = xyz[n - 3U];
		stamp.last[1] = xyz[n - 2U];
		stamp.last[2] = xyz[n - 1U];
	}
	for (const float v : xyz)
	{
		stamp.sum += static_cast<double>(v);
	}
	return stamp;
}

GeomWorldStamp stampFromSnapshot(const TrajectoryGeometrySnapshot& snap)
{
	GeomWorldStamp stamp{};
	if (snap.kind == TrajectoryGeometryKind::TriangleMesh)
	{
		stamp = makeWorldStamp(!snap.triangleSoupModelMm.empty() ? snap.triangleSoupModelMm : snap.triangleSoupWorldMm);
	}
	else if (snap.kind == TrajectoryGeometryKind::PointCloud)
	{
		stamp = makeWorldStamp(!snap.positionsModelMm.empty() ? snap.positionsModelMm : snap.positionsWorldMm);
	}
	if (snap.hasModelToWorld)
	{
		// 位姿变化必须使 SPARE 缓存失效（OSG 序：平移在 v[3,7,11]）
		stamp.sum += snap.modelToWorldColMajor16[3] + snap.modelToWorldColMajor16[7] + snap.modelToWorldColMajor16[11];
		stamp.sum += snap.modelToWorldColMajor16[0] + snap.modelToWorldColMajor16[5] + snap.modelToWorldColMajor16[10];
	}
	return stamp;
}

bool expressGeometryInSourceModelFrame(const TrajectoryGeometrySnapshot& srcSnap,
									   const TrajectoryGeometrySnapshot& tgtSnap,
									   TrajectoryGeometrySnapshot& outTgtInSrc, std::string* errMsg)
{
	if (!srcSnap.hasModelToWorld || !tgtSnap.hasModelToWorld)
	{
		if (errMsg)
		{
			*errMsg = "source/target world matrix unavailable for non-rigid frame";
		}
		return false;
	}
	outTgtInSrc = tgtSnap;
	outTgtInSrc.hasModelToWorld = false;
	const auto xformModelToSrcModel = [&](const float mx, const float my, const float mz, float& ox, float& oy,
										  float& oz)
	{
		const double model[3] = {static_cast<double>(mx), static_cast<double>(my), static_cast<double>(mz)};
		double world[3]{};
		double srcModel[3]{};
		transformPointModelToWorld(tgtSnap.modelToWorldColMajor16, model, world);
		transformPointWorldToModel(srcSnap.modelToWorldColMajor16, world, srcModel);
		ox = static_cast<float>(srcModel[0]);
		oy = static_cast<float>(srcModel[1]);
		oz = static_cast<float>(srcModel[2]);
	};
	if (tgtSnap.kind == TrajectoryGeometryKind::TriangleMesh)
	{
		const std::vector<float>& local =
			!tgtSnap.triangleSoupModelMm.empty() ? tgtSnap.triangleSoupModelMm : tgtSnap.triangleSoupWorldMm;
		outTgtInSrc.triangleSoupWorldMm.clear();
		outTgtInSrc.triangleSoupWorldMm.reserve(local.size());
		for (std::size_t i = 0; i + 2 < local.size(); i += 3)
		{
			float ox = 0.0f;
			float oy = 0.0f;
			float oz = 0.0f;
			xformModelToSrcModel(local[i], local[i + 1], local[i + 2], ox, oy, oz);
			outTgtInSrc.triangleSoupWorldMm.push_back(ox);
			outTgtInSrc.triangleSoupWorldMm.push_back(oy);
			outTgtInSrc.triangleSoupWorldMm.push_back(oz);
		}
		return !outTgtInSrc.triangleSoupWorldMm.empty();
	}
	if (tgtSnap.kind == TrajectoryGeometryKind::PointCloud)
	{
		const std::vector<float>& local =
			!tgtSnap.positionsModelMm.empty() ? tgtSnap.positionsModelMm : tgtSnap.positionsWorldMm;
		outTgtInSrc.positionsWorldMm.clear();
		outTgtInSrc.positionsWorldMm.reserve(local.size());
		for (std::size_t i = 0; i + 2 < local.size(); i += 3)
		{
			float ox = 0.0f;
			float oy = 0.0f;
			float oz = 0.0f;
			xformModelToSrcModel(local[i], local[i + 1], local[i + 2], ox, oy, oz);
			outTgtInSrc.positionsWorldMm.push_back(ox);
			outTgtInSrc.positionsWorldMm.push_back(oy);
			outTgtInSrc.positionsWorldMm.push_back(oz);
		}
		return !outTgtInSrc.positionsWorldMm.empty();
	}
	if (errMsg)
	{
		*errMsg = "unsupported target geometry for non-rigid frame";
	}
	return false;
}

bool stampEqual(const GeomWorldStamp& a, const GeomWorldStamp& b)
{
	return a.floatCount == b.floatCount && a.first[0] == b.first[0] && a.first[1] == b.first[1] &&
		   a.first[2] == b.first[2] && a.last[0] == b.last[0] && a.last[1] == b.last[1] &&
		   a.last[2] == b.last[2] && a.sum == b.sum;
}

bool cacheKeyEqual(const NonRigidSpareCacheKey& a, const NonRigidSpareCacheKey& b)
{
	return a.srcId == b.srcId && a.tgtId == b.tgtId && a.srcKind == b.srcKind && a.tgtKind == b.tgtKind &&
		   stampEqual(a.srcStamp, b.srcStamp) && stampEqual(a.tgtStamp, b.tgtStamp) &&
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
	key.srcStamp = stampFromSnapshot(srcSnap);
	key.tgtStamp = stampFromSnapshot(tgtSnap);
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
						  std::vector<float>& deformedPointCloudOut, point_cloud_backend_ops::PointCloudSpareResult& spareOut,
						  bool& fromCacheOut, std::string* errMsg)
{
	fromCacheOut = false;
	spareOut = point_cloud_backend_ops::PointCloudSpareResult{};
	const NonRigidRegistrationParams effective = withEffectiveSpareParams(srcSnap, params);
	const NonRigidSpareCacheKey key = makeSpareCacheKey(srcSnap, tgtSnap, effective);
	NonRigidSpareCache& cache = nonRigidSpareCache();
	if (cache.valid && cacheKeyEqual(cache.key, key))
	{
		deformedMeshSoupOut = cache.deformedMeshSoup;
		deformedPointCloudOut = cache.deformedPointCloud;
		spareOut.meanErrorMm = cache.meanErrorMm;
		spareOut.deformationNodeCount = cache.deformationNodeCount;
		fromCacheOut = true;
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
	cache.meanErrorMm = spareResult.meanErrorMm;
	cache.deformationNodeCount = spareResult.deformationNodeCount;
	cache.valid = true;
	spareOut = spareResult;
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


bool xyzCentroid3(const std::vector<float>& xyz, double out[3])
{
	out[0] = out[1] = out[2] = 0.0;
	const std::size_t n = xyz.size() / 3U;
	if (n == 0U)
	{
		return false;
	}
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		out[0] += static_cast<double>(xyz[b]);
		out[1] += static_cast<double>(xyz[b + 1U]);
		out[2] += static_cast<double>(xyz[b + 2U]);
	}
	const double inv = 1.0 / static_cast<double>(n);
	out[0] *= inv;
	out[1] *= inv;
	out[2] *= inv;
	return true;
}

bool trajCentroid3(const UnifiedTrajectory& traj, const std::vector<std::size_t>& indices, double out[3])
{
	out[0] = out[1] = out[2] = 0.0;
	std::size_t n = 0;
	for (const std::size_t idx : indices)
	{
		if (idx >= traj.points.size())
		{
			continue;
		}
		out[0] += static_cast<double>(traj.points[idx].poseMm.x);
		out[1] += static_cast<double>(traj.points[idx].poseMm.y);
		out[2] += static_cast<double>(traj.points[idx].poseMm.z);
		++n;
	}
	if (n == 0U)
	{
		return false;
	}
	const double inv = 1.0 / static_cast<double>(n);
	out[0] *= inv;
	out[1] *= inv;
	out[2] *= inv;
	return true;
}

struct NonRigidBindAttempt
{
	std::vector<TrajectoryPointBinding> bindings;
	std::size_t bindOk = 0;
	std::size_t bindFail = 0;
	double bindDistMinMm = 0.0;
	double bindDistMeanMm = 0.0;
	double bindDistMaxMm = 0.0;
};

NonRigidBindAttempt bindTrajectoryToSoup(const UnifiedTrajectory& traj, const std::vector<std::size_t>& indices,
										 const std::vector<float>& soup, const bool sourceIsMesh,
										 const double maxBindDistanceMm)
{
	NonRigidBindAttempt out{};
	out.bindings.reserve(indices.size());
	double bindDistSum = 0.0;
	double bindDistMin = std::numeric_limits<double>::infinity();
	double bindDistMax = 0.0;
	for (const std::size_t idxPoint : indices)
	{
		if (idxPoint >= traj.points.size())
		{
			continue;
		}
		TrajectoryPointBinding item{};
		item.trajIndex = idxPoint;
		item.isMesh = sourceIsMesh;
		const double p[3] = {static_cast<double>(traj.points[idxPoint].poseMm.x),
							 static_cast<double>(traj.points[idxPoint].poseMm.y),
							 static_cast<double>(traj.points[idxPoint].poseMm.z)};
		double bindDist = 0.0;
		if (sourceIsMesh)
		{
			item.meshBinding = bindPointToMeshSoup(p, soup, maxBindDistanceMm);
			bindDist = item.meshBinding.bindDistance;
			if (item.meshBinding.valid)
			{
				++out.bindOk;
			}
			else
			{
				++out.bindFail;
			}
		}
		else
		{
			item.pointBinding = bindPointToPointCloud(p, soup, maxBindDistanceMm);
			bindDist = item.pointBinding.bindDistance;
			if (item.pointBinding.valid)
			{
				++out.bindOk;
			}
			else
			{
				++out.bindFail;
			}
		}
		bindDistSum += bindDist;
		bindDistMin = std::min(bindDistMin, bindDist);
		bindDistMax = std::max(bindDistMax, bindDist);
		out.bindings.push_back(item);
	}
	if (!out.bindings.empty())
	{
		out.bindDistMeanMm = bindDistSum / static_cast<double>(out.bindings.size());
		out.bindDistMinMm = std::isfinite(bindDistMin) ? bindDistMin : 0.0;
		out.bindDistMaxMm = bindDistMax;
	}
	return out;
}

} // namespace

bool nonRigidWarpUnifiedTrajectory(UnifiedTrajectory& traj, const NonRigidRegistrationParams& params,
								   const OpScope& scope, const RobotProgram* program, std::size_t* outMissCount,
								   std::string* errMsg)
{
	g_lastNonRigidStats = NonRigidWarpLastStats{};
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

	clearNonRigidSpareCache();

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
	if (!srcSnap.hasModelToWorld)
	{
		if (errMsg)
		{
			*errMsg = "source world matrix unavailable";
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
	if (!tgtSnap.hasModelToWorld)
	{
		if (errMsg)
		{
			*errMsg = "target world matrix unavailable";
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

	// BREP 工件出轨迹（世界=Tw×模型），非刚性源多为同模离散 mesh（Ts 可能不同）：
	// 绑定在「工件模型系」对源 ModelMm；SPARE 在源模型系；写回 Tw×变形模型（与入管/显示同一世界挂点）
	const bool sourceIsMesh = srcSnap.kind == TrajectoryGeometryKind::TriangleMesh;
	const std::vector<float>& srcModelSoup =
		sourceIsMesh ? (!srcSnap.triangleSoupModelMm.empty() ? srcSnap.triangleSoupModelMm : srcSnap.triangleSoupWorldMm)
					 : (!srcSnap.positionsModelMm.empty() ? srcSnap.positionsModelMm : srcSnap.positionsWorldMm);
	const std::vector<float>& srcWorldSoup =
		sourceIsMesh ? srcSnap.triangleSoupWorldMm : srcSnap.positionsWorldMm;
	if (srcModelSoup.empty() && srcWorldSoup.empty())
	{
		if (errMsg)
		{
			*errMsg = "source geometry is empty";
		}
		return false;
	}

	RawTrajectory trajMeta{};
	trajMeta.sourceFeatureJson = traj.sourceFeatureJson;
	const std::string trajWp = rawTrajectoryWorkpieceBackendId(trajMeta);

	bool bindInWorkpieceModel = false;
	TrajectoryGeometrySnapshot wpSnap{};
	if (!trajWp.empty() && trajWp != params.sourceBackendId)
	{
		std::string wpErr;
		if (resolveTrajectoryGeometry(trajWp, wpSnap, &wpErr) && wpSnap.hasModelToWorld)
		{
			bindInWorkpieceModel = true;
		}
		else
		{
			RunLogger::warn(std::string("[非刚性配准] 无法解析轨迹工件矩阵 backend=") + trajWp + " err=" + wpErr);
		}
	}

	UnifiedTrajectory trajForBind = traj;
	if (bindInWorkpieceModel)
	{
		for (const std::size_t idxPoint : indices)
		{
			if (idxPoint >= trajForBind.points.size())
			{
				continue;
			}
			UnifiedTrajectoryPoint& p = trajForBind.points[idxPoint];
			const double worldP[3] = {static_cast<double>(p.poseMm.x), static_cast<double>(p.poseMm.y),
									 static_cast<double>(p.poseMm.z)};
			double modelP[3]{};
			transformPointWorldToModel(wpSnap.modelToWorldColMajor16, worldP, modelP);
			p.poseMm.x = static_cast<float>(modelP[0]);
			p.poseMm.y = static_cast<float>(modelP[1]);
			p.poseMm.z = static_cast<float>(modelP[2]);
		}
	}

	NonRigidBindAttempt bind{};
	if (bindInWorkpieceModel)
	{
		bind = bindTrajectoryToSoup(trajForBind, indices, srcModelSoup, sourceIsMesh, params.maxBindDistanceMm);
	}
	else
	{
		bind = bindTrajectoryToSoup(traj, indices, srcWorldSoup, sourceIsMesh, params.maxBindDistanceMm);
	}
	if (bind.bindings.empty())
	{
		if (errMsg)
		{
			*errMsg = "no trajectory points in scope";
		}
		return false;
	}

	{
		double trajC[3]{};
		double srcC[3]{};
		double tgtC[3]{};
		const std::vector<float>& tgtWorldSoup = tgtSnap.kind == TrajectoryGeometryKind::TriangleMesh
													? tgtSnap.triangleSoupWorldMm
													: tgtSnap.positionsWorldMm;
		(void)trajCentroid3(traj, indices, trajC);
		(void)xyzCentroid3(srcWorldSoup, srcC);
		(void)xyzCentroid3(tgtWorldSoup, tgtC);
		std::ostringstream os;
		os << "[非刚性配准] 诊断 source=" << params.sourceBackendId << " target=" << params.targetBackendId;
		if (!trajWp.empty())
		{
			os << " 轨迹工件=" << trajWp;
		}
		os << " 绑定模式=" << (bindInWorkpieceModel ? "工件模型系→源Model" : "世界系→源World")
		   << " 轨迹质心W=(" << trajC[0] << "," << trajC[1] << "," << trajC[2] << ")"
		   << " 源世界质心=(" << srcC[0] << "," << srcC[1] << "," << srcC[2] << ")"
		   << " 目标世界质心=(" << tgtC[0] << "," << tgtC[1] << "," << tgtC[2] << ")"
		   << " 绑定成功=" << bind.bindOk << "/" << (bind.bindOk + bind.bindFail)
		   << " meanDist=" << bind.bindDistMeanMm << " mm";
		RunLogger::info(os.str());
	}
	if (bind.bindOk == 0)
	{
		std::ostringstream fail;
		fail << "non-rigid bind failed: meanDist=" << bind.bindDistMeanMm << " mm"
			 << " (mode=" << (bindInWorkpieceModel ? "workpieceModel" : "world") << ")";
		if (errMsg)
		{
			*errMsg = fail.str();
		}
		RunLogger::warn(std::string("[非刚性配准] ") + fail.str());
		return false;
	}

	TrajectoryGeometrySnapshot srcForSpare = srcSnap;
	TrajectoryGeometrySnapshot tgtForSpare = tgtSnap;
	if (bindInWorkpieceModel)
	{
		if (sourceIsMesh)
		{
			srcForSpare.triangleSoupWorldMm = srcModelSoup;
		}
		else
		{
			srcForSpare.positionsWorldMm = srcModelSoup;
		}
		if (!expressGeometryInSourceModelFrame(srcSnap, tgtSnap, tgtForSpare, errMsg))
		{
			return false;
		}
	}

	std::vector<float> deformedMeshSoup;
	std::vector<float> deformedPointCloud;
	point_cloud_backend_ops::PointCloudSpareResult spareResult;
	bool spareFromCache = false;
	if (!runSpareRegistration(srcForSpare, tgtForSpare, params, deformedMeshSoup, deformedPointCloud, spareResult,
							  spareFromCache, errMsg))
	{
		return false;
	}

	std::size_t missCount = 0;
	for (const TrajectoryPointBinding& item : bind.bindings)
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
		if (bindInWorkpieceModel)
		{
			// 与入管一致：轨迹世界系挂在 BREP 工件 Tw 上，不能用源 mesh 的 Ts（二者常不一致）
			const double modelOut[3] = {static_cast<double>(point.poseMm.x), static_cast<double>(point.poseMm.y),
										static_cast<double>(point.poseMm.z)};
			double worldOut[3]{};
			transformPointModelToWorld(wpSnap.modelToWorldColMajor16, modelOut, worldOut);
			point.poseMm.x = static_cast<float>(worldOut[0]);
			point.poseMm.y = static_cast<float>(worldOut[1]);
			point.poseMm.z = static_cast<float>(worldOut[2]);
		}
	}
	if (outMissCount)
	{
		*outMissCount = missCount;
	}

	g_lastNonRigidStats.valid = true;
	g_lastNonRigidStats.bindOk = bind.bindOk;
	g_lastNonRigidStats.bindFail = bind.bindFail;
	g_lastNonRigidStats.bindDistMinMm = bind.bindDistMinMm;
	g_lastNonRigidStats.bindDistMeanMm = bind.bindDistMeanMm;
	g_lastNonRigidStats.bindDistMaxMm = bind.bindDistMaxMm;
	g_lastNonRigidStats.bindMode = bindInWorkpieceModel ? 1 : 0;
	g_lastNonRigidStats.meanErrorMm = spareResult.meanErrorMm;
	g_lastNonRigidStats.deformationNodeCount = spareResult.deformationNodeCount;
	g_lastNonRigidStats.spareFromCache = spareFromCache;

	{
		double outC[3]{};
		(void)trajCentroid3(traj, indices, outC);
		std::ostringstream os;
		os << "[非刚性配准] 绑定成功=" << bind.bindOk << " 失败=" << bind.bindFail
		   << " 模式=" << (bindInWorkpieceModel ? "工件模型系" : "世界系")
		   << " 距离min/mean/max=" << bind.bindDistMinMm << "/" << bind.bindDistMeanMm << "/" << bind.bindDistMaxMm
		   << " mm SPARE均值误差=" << spareResult.meanErrorMm << " mm 变形节点=" << spareResult.deformationNodeCount
		   << (spareFromCache ? " (缓存)" : "") << " 写回后轨迹质心W=(" << outC[0] << "," << outC[1] << "," << outC[2]
		   << ")";
		RunLogger::info(os.str());
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
