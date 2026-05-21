#pragma once

#include "point_cloud_algorithm_global.h"

#include <Eigen/Geometry>

#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API void cropXyzByBox(
	const std::vector<float>& srcXyz,
	const Eigen::AlignedBox3d& box,
	std::vector<float>& outXyz,
	std::vector<std::size_t>* keptIndices = nullptr);

POINT_CLOUD_ALGORITHM_API void cropXyzByBox(
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcRgba,
	const Eigen::AlignedBox3d& box,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::vector<std::size_t>* keptIndices = nullptr);

POINT_CLOUD_ALGORITHM_API void cropXyzBySphere(
	const std::vector<float>& srcXyz,
	const Eigen::Vector3d& centerMm,
	double radiusMm,
	std::vector<float>& outXyz,
	std::vector<std::size_t>* keptIndices = nullptr);

} // namespace pclalgo
