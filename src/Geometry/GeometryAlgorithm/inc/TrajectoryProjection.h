#pragma once

#include "geometry_algorithm_global.h"

#include <cstddef>
#include <vector>

namespace geoalgo
{

/// 沿单位方向射线投影到三角 soup（世界 mm，每三角 9 float）
GEOMETRY_ALGORITHM_API bool projectRayOntoTriangleSoup(
	const double originWorldMm[3],
	const double dirUnit[3],
	double maxDistanceMm,
	const std::vector<float>& triangleSoupWorldMm,
	double outHitWorldMm[3],
	bool& outHit);

/// 沿射线在点云（世界 mm，3N）上找最近命中（管道半径 mm）
GEOMETRY_ALGORITHM_API bool projectRayOntoPointCloud(
	const double originWorldMm[3],
	const double dirUnit[3],
	double maxDistanceMm,
	double hitRadiusMm,
	const std::vector<float>& positionsWorldMm,
	double outHitWorldMm[3],
	bool& outHit);

} // namespace geoalgo
