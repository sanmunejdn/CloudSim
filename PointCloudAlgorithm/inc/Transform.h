#pragma once

#include "point_cloud_algorithm_global.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API void transformXyzInPlace(std::vector<float>& xyz, const Eigen::Isometry3d& t);
POINT_CLOUD_ALGORITHM_API void transformXyz(
	const float* srcXyz,
	std::size_t pointCount,
	const Eigen::Isometry3d& t,
	std::vector<float>& outXyz);

} // namespace pclalgo
