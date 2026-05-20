#pragma once

#include "point_cloud_algorithm_global.h"

#include <Eigen/Geometry>

#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API Eigen::AlignedBox3d computeBoundingBox(const std::vector<float>& xyz);
POINT_CLOUD_ALGORITHM_API Eigen::Vector3d computeCentroid(const std::vector<float>& xyz);
POINT_CLOUD_ALGORITHM_API double computeAverageSpacingMm(const std::vector<float>& xyz, unsigned int kNeighbors = 6);

} // namespace pclalgo
