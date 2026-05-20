#pragma once

#include "point_cloud_algorithm_global.h"

#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API bool downsampleVoxelGrid(
	std::vector<float>& xyzInOut,
	double voxelSizeMm,
	unsigned int minPointsPerCell = 1,
	std::vector<float>* rgbaInOut = nullptr);

POINT_CLOUD_ALGORITHM_API bool downsampleRandom(
	std::vector<float>& xyzInOut,
	double retainedFraction,
	std::vector<float>* rgbaInOut = nullptr);

} // namespace pclalgo
