/// @file TrajectoryProjection.cpp
/// @brief TrajectoryProjection 实现

#include "TrajectoryProjection.h"

#include <cmath>
#include <limits>

namespace geoalgo
{
namespace
{
bool rayTriangleIntersect(const double origin[3], const double dir[3], const double v0[3], const double v1[3],
						  const double v2[3], double& outT)
{
	const double e1[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
	const double e2[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
	const double p[3] = {dir[1] * e2[2] - dir[2] * e2[1], dir[2] * e2[0] - dir[0] * e2[2],
						 dir[0] * e2[1] - dir[1] * e2[0]};
	const double det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
	if (std::fabs(det) < 1e-12)
	{
		return false;
	}
	const double invDet = 1.0 / det;
	const double t0[3] = {origin[0] - v0[0], origin[1] - v0[1], origin[2] - v0[2]};
	const double u = (t0[0] * p[0] + t0[1] * p[1] + t0[2] * p[2]) * invDet;
	if (u < 0.0 || u > 1.0)
	{
		return false;
	}
	const double q[3] = {t0[1] * e1[2] - t0[2] * e1[1], t0[2] * e1[0] - t0[0] * e1[2], t0[0] * e1[1] - t0[1] * e1[0]};
	const double v = (dir[0] * q[0] + dir[1] * q[1] + dir[2] * q[2]) * invDet;
	if (v < 0.0 || u + v > 1.0)
	{
		return false;
	}
	const double t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * invDet;
	if (t < 1e-9)
	{
		return false;
	}
	outT = t;
	return true;
}

} // namespace

bool projectRayOntoTriangleSoup(const double originWorldMm[3], const double dirUnit[3], const double maxDistanceMm,
								const std::vector<float>& triangleSoupWorldMm, double outHitWorldMm[3], bool& outHit)
{
	outHit = false;
	if (triangleSoupWorldMm.size() < 9)
	{
		return true;
	}
	const size_t triCount = triangleSoupWorldMm.size() / 9;
	double bestT = std::numeric_limits<double>::max();
	for (size_t t = 0; t < triCount; ++t)
	{
		const size_t base = t * 9;
		const double v0[3] = {triangleSoupWorldMm[base + 0], triangleSoupWorldMm[base + 1],
							  triangleSoupWorldMm[base + 2]};
		const double v1[3] = {triangleSoupWorldMm[base + 3], triangleSoupWorldMm[base + 4],
							  triangleSoupWorldMm[base + 5]};
		const double v2[3] = {triangleSoupWorldMm[base + 6], triangleSoupWorldMm[base + 7],
							  triangleSoupWorldMm[base + 8]};
		double tHit = 0.0;
		if (!rayTriangleIntersect(originWorldMm, dirUnit, v0, v1, v2, tHit))
		{
			continue;
		}
		if (tHit > maxDistanceMm || tHit >= bestT)
		{
			continue;
		}
		bestT = tHit;
	}
	if (bestT < std::numeric_limits<double>::max())
	{
		outHit = true;
		outHitWorldMm[0] = originWorldMm[0] + dirUnit[0] * bestT;
		outHitWorldMm[1] = originWorldMm[1] + dirUnit[1] * bestT;
		outHitWorldMm[2] = originWorldMm[2] + dirUnit[2] * bestT;
	}
	return true;
}

bool projectRayOntoPointCloud(const double originWorldMm[3], const double dirUnit[3], const double maxDistanceMm,
							  const double hitRadiusMm, const std::vector<float>& positionsWorldMm,
							  double outHitWorldMm[3], bool& outHit)
{
	outHit = false;
	if (positionsWorldMm.size() < 3)
	{
		return true;
	}
	const size_t pointCount = positionsWorldMm.size() / 3;
	const double radiusSq = hitRadiusMm * hitRadiusMm;
	double bestT = std::numeric_limits<double>::max();
	for (size_t i = 0; i < pointCount; ++i)
	{
		const size_t base = i * 3;
		const double px = positionsWorldMm[base + 0];
		const double py = positionsWorldMm[base + 1];
		const double pz = positionsWorldMm[base + 2];
		const double ox = px - originWorldMm[0];
		const double oy = py - originWorldMm[1];
		const double oz = pz - originWorldMm[2];
		const double t = ox * dirUnit[0] + oy * dirUnit[1] + oz * dirUnit[2];
		if (t < 0.0 || t > maxDistanceMm || t >= bestT)
		{
			continue;
		}
		const double cx = originWorldMm[0] + dirUnit[0] * t;
		const double cy = originWorldMm[1] + dirUnit[1] * t;
		const double cz = originWorldMm[2] + dirUnit[2] * t;
		const double dx = px - cx;
		const double dy = py - cy;
		const double dz = pz - cz;
		if (dx * dx + dy * dy + dz * dz <= radiusSq)
		{
			bestT = t;
			outHitWorldMm[0] = px;
			outHitWorldMm[1] = py;
			outHitWorldMm[2] = pz;
		}
	}
	if (bestT < std::numeric_limits<double>::max())
	{
		outHit = true;
	}
	return true;
}

} // namespace geoalgo
