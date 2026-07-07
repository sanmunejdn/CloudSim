#pragma once

#include "point_cloud_algorithm_global.h"

#include <vector>

namespace pclalgo
{

constexpr std::size_t kFpfhDim = 33U;

POINT_CLOUD_ALGORITHM_API void computeSpfhForCloud(
	const std::vector<float>& xyz,
	const std::vector<float>& normals,
	const unsigned int kNeighbors,
	std::vector<float>& outSpfh);

POINT_CLOUD_ALGORITHM_API void computeFpfhForCloud(
	const std::vector<float>& xyz,
	const std::vector<float>& normals,
	const std::vector<float>& spfh,
	const unsigned int kNeighbors,
	std::vector<float>& outFpfh);

POINT_CLOUD_ALGORITHM_API float fpfhL2Distance(const float* a, const float* b);

} // namespace pclalgo
