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

/// 屏幕多边形裁剪：点先经 modelToWorld 再乘 mvp 投影，射线法判多边形内外
POINT_CLOUD_ALGORITHM_API void cropXyzByPolyline2D(
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcRgba,
	const std::vector<float>& polylineScreenXy,
	const double mvpMatrix[16],
	const double modelToWorld[16],
	int viewportWidth,
	int viewportHeight,
	bool keepInside,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::vector<std::size_t>* keptIndices = nullptr);

} // namespace pclalgo
