#ifndef GEOMETRYALGORITHM_TRAJECTORYPROJECTION_H
#define GEOMETRYALGORITHM_TRAJECTORYPROJECTION_H

/// @file TrajectoryProjection.h
/// @brief 沿射线将轨迹点投影到三角 soup 或点云（世界坐标 mm）

#include "geometry_algorithm_global.h"

#include <cstddef>
#include <vector>

namespace geoalgo
{
/**
 * 射线与三角 soup 求最近交点
 * @param originWorldMm 射线起点（mm）
 * @param dirUnit 单位方向
 * @param maxDistanceMm 最大搜索距离（mm）
 * @param triangleSoupWorldMm 9T float
 * @param outHit 是否命中
 * @return false：soup 布局非法
 */
GEOMETRY_ALGORITHM_API bool projectRayOntoTriangleSoup(const double originWorldMm[3], const double dirUnit[3],
													   double maxDistanceMm,
													   const std::vector<float>& triangleSoupWorldMm,
													   double outHitWorldMm[3], bool& outHit);

/**
 * 射线在点云上找最近命中（圆柱管半径近似）
 * @param hitRadiusMm 命中判定半径（mm）
 * @param positionsWorldMm 3N float
 * @return false：点云为空
 */
GEOMETRY_ALGORITHM_API bool projectRayOntoPointCloud(const double originWorldMm[3], const double dirUnit[3],
													 double maxDistanceMm, double hitRadiusMm,
													 const std::vector<float>& positionsWorldMm,
													 double outHitWorldMm[3], bool& outHit);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_TRAJECTORYPROJECTION_H
