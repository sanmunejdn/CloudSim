#ifndef POINTCLOUDALGORITHM_MEASURE_H
#define POINTCLOUDALGORITHM_MEASURE_H

/// @file Measure.h
/// @brief Measure 接口

#include "point_cloud_algorithm_global.h"

#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
POINT_CLOUD_ALGORITHM_API Eigen::AlignedBox3d computeBoundingBox(const std::vector<float>& xyz);
POINT_CLOUD_ALGORITHM_API Eigen::Vector3d computeCentroid(const std::vector<float>& xyz);
POINT_CLOUD_ALGORITHM_API double computeAverageSpacingMm(const std::vector<float>& xyz, unsigned int kNeighbors = 6);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_MEASURE_H
