#ifndef POINTCLOUDALGORITHM_TRANSFORM_H
#define POINTCLOUDALGORITHM_TRANSFORM_H

/// @file Transform.h
/// @brief Transform 接口

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
POINT_CLOUD_ALGORITHM_API void transformXyzInPlace(std::vector<float>& xyz, const Eigen::Isometry3d& t);
POINT_CLOUD_ALGORITHM_API void transformXyz(const float* srcXyz, std::size_t pointCount, const Eigen::Isometry3d& t,
											std::vector<float>& outXyz);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_TRANSFORM_H
