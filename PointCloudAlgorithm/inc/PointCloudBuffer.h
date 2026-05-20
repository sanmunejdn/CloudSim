#pragma once

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API std::size_t pointCountFromXyz(const std::vector<float>& xyz);
POINT_CLOUD_ALGORITHM_API std::size_t triangleCountFromSoup(const std::vector<float>& triangleSoup);
POINT_CLOUD_ALGORITHM_API bool validXyzLength(const std::vector<float>& xyz);
POINT_CLOUD_ALGORITHM_API bool validRgbaLength(const std::vector<float>& rgba, std::size_t pointCount);

POINT_CLOUD_ALGORITHM_API void compactXyzByIndices(
	const std::vector<float>& srcXyz,
	const std::vector<std::size_t>& keepIndices,
	std::vector<float>& outXyz);

POINT_CLOUD_ALGORITHM_API void compactRgbaByIndices(
	const std::vector<float>& srcRgba,
	const std::vector<std::size_t>& keepIndices,
	std::vector<float>& outRgba);

} // namespace pclalgo
