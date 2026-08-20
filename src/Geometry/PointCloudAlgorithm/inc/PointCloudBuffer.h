#ifndef POINTCLOUDALGORITHM_POINTCLOUDBUFFER_H
#define POINTCLOUDALGORITHM_POINTCLOUDBUFFER_H

/// @file PointCloudBuffer.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 点云/soup 缓冲校验与按索引紧凑拷贝

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <vector>

namespace pclalgo
{
/** xyz 长度须为 3 的倍数；返回点数 */
POINT_CLOUD_ALGORITHM_API std::size_t pointCountFromXyz(const std::vector<float>& xyz);

/** triangleSoup 长度须为 9 的倍数；返回三角数 */
POINT_CLOUD_ALGORITHM_API std::size_t triangleCountFromSoup(const std::vector<float>& triangleSoup);

POINT_CLOUD_ALGORITHM_API bool validXyzLength(const std::vector<float>& xyz);
POINT_CLOUD_ALGORITHM_API bool validRgbaLength(const std::vector<float>& rgba, std::size_t pointCount);

POINT_CLOUD_ALGORITHM_API void compactXyzByIndices(const std::vector<float>& srcXyz,
												   const std::vector<std::size_t>& keepIndices,
												   std::vector<float>& outXyz);

POINT_CLOUD_ALGORITHM_API void compactRgbaByIndices(const std::vector<float>& srcRgba,
													const std::vector<std::size_t>& keepIndices,
													std::vector<float>& outRgba);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_POINTCLOUDBUFFER_H
