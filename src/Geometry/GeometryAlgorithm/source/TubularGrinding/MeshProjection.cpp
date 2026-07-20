/// @file MeshProjection.cpp
/// @brief MeshProjection 实现

#include "MeshProjection.h"

#include "TrajectoryProjection.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <KdTreePointSet.h>

namespace geoalgo
{
namespace tg
{
namespace
{
bool rayHitTriangleSoup(const std::vector<float>& soup, const double origin[3], const double dir[3],
						const double maxDist, double outHit[3])
{
	bool hit = false;
	if (!projectRayOntoTriangleSoup(origin, dir, maxDist, soup, outHit, hit))
	{
		return false;
	}
	return hit;
}

Vec3 triangleNormalAtHit(const std::vector<float>& soup, const double hit[3])
{
	const int faceCount = static_cast<int>(soup.size() / 9U);
	double bestD2 = 1e30;
	Vec3 bestN{0.0, 0.0, 1.0};
	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		Vec3 c{(soup[base + 0] + soup[base + 3] + soup[base + 6]) / 3.0f,
			   (soup[base + 1] + soup[base + 4] + soup[base + 7]) / 3.0f,
			   (soup[base + 2] + soup[base + 5] + soup[base + 8]) / 3.0f};
		const double dx = hit[0] - c.x;
		const double dy = hit[1] - c.y;
		const double dz = hit[2] - c.z;
		const double d2 = dx * dx + dy * dy + dz * dz;
		if (d2 < bestD2)
		{
			bestD2 = d2;
			Vec3 v0{soup[base + 0], soup[base + 1], soup[base + 2]};
			Vec3 v1{soup[base + 3], soup[base + 4], soup[base + 5]};
			Vec3 v2{soup[base + 6], soup[base + 7], soup[base + 8]};
			bestN = normalizeVec3(cross(sub(v1, v0), sub(v2, v0)));
		}
	}
	return bestN;
}

} // namespace

bool runMeshProjection(const IndexedMeshLite& mesh, const std::vector<TubularTemplatePoint>& templatePoints,
					   const TubularGrindingParams& params, std::vector<TubularProjectedPoint>& outPoints,
					   double& outHitRate, std::string* errMsg)
{
	outPoints.clear();
	outHitRate = 0.0;
	if (templatePoints.empty())
	{
		if (errMsg)
		{
			*errMsg = "no template points";
		}
		return false;
	}

	int hitCount = 0;
	for (const TubularTemplatePoint& tp : templatePoints)
	{
		const double origin[3] = {tp.positionMm[0], tp.positionMm[1], tp.positionMm[2]};
		const double n[3] = {tp.normalMm[0], tp.normalMm[1], tp.normalMm[2]};
		double dirPos[3] = {n[0], n[1], n[2]};
		double dirNeg[3] = {-n[0], -n[1], -n[2]};
		const double lenN = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
		if (lenN > 1e-9)
		{
			dirPos[0] /= lenN;
			dirPos[1] /= lenN;
			dirPos[2] /= lenN;
			dirNeg[0] /= lenN;
			dirNeg[1] /= lenN;
			dirNeg[2] /= lenN;
		}

		double hitPos[3] = {0.0, 0.0, 0.0};
		double hitNeg[3] = {0.0, 0.0, 0.0};
		const bool hasPos = rayHitTriangleSoup(mesh.soup, origin, dirPos, params.projectionMaxDistMm, hitPos);
		const bool hasNeg = rayHitTriangleSoup(mesh.soup, origin, dirNeg, params.projectionMaxDistMm, hitNeg);

		TubularProjectedPoint pp;
		pp.pipeId = tp.pipeId;
		if (!hasPos && !hasNeg)
		{
			pp.positionMm = tp.positionMm;
			pp.normalMm = tp.normalMm;
		}
		else
		{
			double bestHit[3];
			if (hasPos && hasNeg)
			{
				const double dPos = (hitPos[0] - origin[0]) * (hitPos[0] - origin[0]) +
									(hitPos[1] - origin[1]) * (hitPos[1] - origin[1]) +
									(hitPos[2] - origin[2]) * (hitPos[2] - origin[2]);
				const double dNeg = (hitNeg[0] - origin[0]) * (hitNeg[0] - origin[0]) +
									(hitNeg[1] - origin[1]) * (hitNeg[1] - origin[1]) +
									(hitNeg[2] - origin[2]) * (hitNeg[2] - origin[2]);
				if (dPos <= dNeg)
				{
					bestHit[0] = hitPos[0];
					bestHit[1] = hitPos[1];
					bestHit[2] = hitPos[2];
				}
				else
				{
					bestHit[0] = hitNeg[0];
					bestHit[1] = hitNeg[1];
					bestHit[2] = hitNeg[2];
				}
			}
			else if (hasPos)
			{
				bestHit[0] = hitPos[0];
				bestHit[1] = hitPos[1];
				bestHit[2] = hitPos[2];
			}
			else
			{
				bestHit[0] = hitNeg[0];
				bestHit[1] = hitNeg[1];
				bestHit[2] = hitNeg[2];
			}
			const Vec3 faceN = triangleNormalAtHit(mesh.soup, bestHit);
			pp.positionMm = {bestHit[0], bestHit[1], bestHit[2]};
			pp.normalMm = {faceN.x, faceN.y, faceN.z};
			++hitCount;
		}
		outPoints.push_back(pp);
	}

	outHitRate = static_cast<double>(hitCount) / static_cast<double>(templatePoints.size());
	return !outPoints.empty();
}

bool runPointCloudProjection(const std::vector<float>& pointXyz,
							 const std::vector<TubularTemplatePoint>& templatePoints,
							 const TubularGrindingParams& params, std::vector<TubularProjectedPoint>& outPoints,
							 double& outHitRate, std::string* errMsg)
{
	outPoints.clear();
	outHitRate = 0.0;
	const std::size_t pointCount = pointXyz.size() / 3U;
	if (templatePoints.empty() || pointCount == 0U)
	{
		if (errMsg)
		{
			*errMsg = templatePoints.empty() ? "no template points" : "empty point cloud";
		}
		return false;
	}

	pclalgo::KdTreePointSet kdTree(pointXyz);
	if (kdTree.empty())
	{
		if (errMsg)
		{
			*errMsg = "failed to build point cloud kd-tree";
		}
		return false;
	}

	const double maxDist = params.projectionMaxDistMm > 0.0 ? params.projectionMaxDistMm : 10.0;
	const double maxDist2 = maxDist * maxDist;
	const unsigned int queryK = static_cast<unsigned int>(std::min(static_cast<std::size_t>(200), pointCount));
	int hitCount = 0;

	for (const TubularTemplatePoint& tp : templatePoints)
	{
		const double ox = tp.positionMm[0];
		const double oy = tp.positionMm[1];
		const double oz = tp.positionMm[2];
		double nx = tp.normalMm[0];
		double ny = tp.normalMm[1];
		double nz = tp.normalMm[2];
		const double lenN = std::sqrt(nx * nx + ny * ny + nz * nz);
		if (lenN > 1e-9)
		{
			nx /= lenN;
			ny /= lenN;
			nz /= lenN;
		}
		else
		{
			nx = 0.0;
			ny = 0.0;
			nz = 1.0;
		}

		double bestDist2Pos = maxDist2;
		double bestDist2Neg = maxDist2;
		double bestPos[3] = {0.0, 0.0, 0.0};
		double bestNeg[3] = {0.0, 0.0, 0.0};
		bool hasPos = false;
		bool hasNeg = false;

		std::vector<std::size_t> idx;
		std::vector<double> distSq;
		kdTree.findKNearest(ox, oy, oz, queryK, idx, distSq);

		for (std::size_t ki = 0; ki < idx.size(); ++ki)
		{
			const std::size_t pi = idx[ki];
			const double px = static_cast<double>(pointXyz[pi * 3U + 0U]);
			const double py = static_cast<double>(pointXyz[pi * 3U + 1U]);
			const double pz = static_cast<double>(pointXyz[pi * 3U + 2U]);
			const double vx = px - ox;
			const double vy = py - oy;
			const double vz = pz - oz;
			const double proj = vx * nx + vy * ny + vz * nz;
			const double dist2 = vx * vx + vy * vy + vz * vz;
			const double perp2 = dist2 - proj * proj;

			if (perp2 > maxDist2 * 0.25)
			{
				continue;
			}

			if (proj > 0.0 && proj <= maxDist && dist2 <= bestDist2Pos)
			{
				bestDist2Pos = dist2;
				bestPos[0] = px;
				bestPos[1] = py;
				bestPos[2] = pz;
				hasPos = true;
			}

			if (proj < 0.0 && -proj <= maxDist && dist2 <= bestDist2Neg)
			{
				bestDist2Neg = dist2;
				bestNeg[0] = px;
				bestNeg[1] = py;
				bestNeg[2] = pz;
				hasNeg = true;
			}
		}

		TubularProjectedPoint pp;
		pp.pipeId = tp.pipeId;
		if (!hasPos && !hasNeg)
		{
			pp.positionMm = tp.positionMm;
			pp.normalMm = tp.normalMm;
		}
		else
		{
			const double* bestHit = (hasPos && (!hasNeg || bestDist2Pos <= bestDist2Neg)) ? bestPos : bestNeg;
			pp.positionMm = {bestHit[0], bestHit[1], bestHit[2]};
			pp.normalMm = tp.normalMm;
			++hitCount;
		}
		outPoints.push_back(pp);
	}

	outHitRate = static_cast<double>(hitCount) / static_cast<double>(templatePoints.size());
	return !outPoints.empty();
}

} // namespace tg
} // namespace geoalgo
