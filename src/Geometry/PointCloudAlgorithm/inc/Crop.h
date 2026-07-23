#ifndef POINTCLOUDALGORITHM_CROP_H
#define POINTCLOUDALGORITHM_CROP_H

/// @file Crop.h
/// @brief 点云裁剪：AABB / 球 / 屏幕多边形

#include "point_cloud_algorithm_global.h"

#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
/**
 * 轴对齐盒裁剪（模型/世界系与 box 一致即可）
 * @param keptIndices 可选，保留点在源中的下标
 */
POINT_CLOUD_ALGORITHM_API void cropXyzByBox(const std::vector<float>& srcXyz, const Eigen::AlignedBox3d& box,
											std::vector<float>& outXyz,
											std::vector<std::size_t>* keptIndices = nullptr);

/** 盒裁剪并同步 rgba（4*N，0..1） */
POINT_CLOUD_ALGORITHM_API void cropXyzByBox(const std::vector<float>& srcXyz, const std::vector<float>& srcRgba,
											const Eigen::AlignedBox3d& box, std::vector<float>& outXyz,
											std::vector<float>& outRgba,
											std::vector<std::size_t>* keptIndices = nullptr);

/**
 * 球裁剪
 * @param centerMm 球心 mm
 * @param radiusMm 半径 mm；≤0 通常无点保留
 */
POINT_CLOUD_ALGORITHM_API void cropXyzBySphere(const std::vector<float>& srcXyz, const Eigen::Vector3d& centerMm,
											   double radiusMm, std::vector<float>& outXyz,
											   std::vector<std::size_t>* keptIndices = nullptr);

/**
 * 屏幕多边形索引收集：点经 modelToWorld 再乘 mvp 投影，射线法判内外
 * @param polylineScreenXy 屏幕像素折线，2*M
 * @param keepInside true=保留多边形内
 */
POINT_CLOUD_ALGORITHM_API void collectXyzIndicesByPolyline2D(const std::vector<float>& srcXyz,
															 const std::vector<float>& polylineScreenXy,
															 const double mvpMatrix[16], const double modelToWorld[16],
															 int viewportWidth, int viewportHeight, bool keepInside,
															 std::vector<std::size_t>& outIndices);

/** 屏幕多边形裁剪并同步 rgba；投影规则同 collectXyzIndicesByPolyline2D */
POINT_CLOUD_ALGORITHM_API void cropXyzByPolyline2D(const std::vector<float>& srcXyz, const std::vector<float>& srcRgba,
												   const std::vector<float>& polylineScreenXy,
												   const double mvpMatrix[16], const double modelToWorld[16],
												   int viewportWidth, int viewportHeight, bool keepInside,
												   std::vector<float>& outXyz, std::vector<float>& outRgba,
												   std::vector<std::size_t>* keptIndices = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_CROP_H
