#include "MeshProjection.h"

#include "TrajectoryProjection.h"

#include <cmath>
#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{

namespace
{

bool rayHitTriangleSoup(
	const std::vector<float>& soup,
	const double origin[3],
	const double dir[3],
	const double maxDist,
	double outHit[3])
{
	bool hit = false;
	if (!projectRayOntoTriangleSoup(origin, dir, maxDist, soup, outHit, hit))
	{
		return false;
	}
	return hit;
}

Vec3 triangleNormalAtHit(
	const std::vector<float>& soup,
	const double hit[3])
{
	const int faceCount = static_cast<int>(soup.size() / 9U);
	double bestD2 = 1e30;
	Vec3 bestN{0.0, 0.0, 1.0};
	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		Vec3 c{
			(soup[base + 0] + soup[base + 3] + soup[base + 6]) / 3.0f,
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

bool runMeshProjection(
	const IndexedMeshLite& mesh,
	const std::vector<TubularTemplatePoint>& templatePoints,
	const TubularGrindingParams& params,
	std::vector<TubularProjectedPoint>& outPoints,
	double& outHitRate,
	std::string* errMsg)
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
				const double dPos = (hitPos[0] - origin[0]) * (hitPos[0] - origin[0])
					+ (hitPos[1] - origin[1]) * (hitPos[1] - origin[1])
					+ (hitPos[2] - origin[2]) * (hitPos[2] - origin[2]);
				const double dNeg = (hitNeg[0] - origin[0]) * (hitNeg[0] - origin[0])
					+ (hitNeg[1] - origin[1]) * (hitNeg[1] - origin[1])
					+ (hitNeg[2] - origin[2]) * (hitNeg[2] - origin[2]);
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

} // namespace tg
} // namespace geoalgo
