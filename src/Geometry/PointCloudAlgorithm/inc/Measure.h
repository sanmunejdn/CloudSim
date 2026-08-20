#ifndef POINTCLOUDALGORITHM_MEASURE_H
#define POINTCLOUDALGORITHM_MEASURE_H

/// @file Measure.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 点云度量：包围盒、质心、平均点距

#include "point_cloud_algorithm_global.h"

#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
/** @param xyz 3*N float，mm；空或非法长度时返回空盒 */
POINT_CLOUD_ALGORITHM_API Eigen::AlignedBox3d computeBoundingBox(const std::vector<float>& xyz);

/** @param xyz 3*N float，mm；空则返回零向量 */
POINT_CLOUD_ALGORITHM_API Eigen::Vector3d computeCentroid(const std::vector<float>& xyz);

/**
 * CGAL 平均点距（mm）
 * @param kNeighbors 邻域点数，默认 6
 * @return ≤0 表示无法估计（点数过少等）
 */
POINT_CLOUD_ALGORITHM_API double computeAverageSpacingMm(const std::vector<float>& xyz, unsigned int kNeighbors = 6);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_MEASURE_H
