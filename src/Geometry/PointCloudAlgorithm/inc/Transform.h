#ifndef POINTCLOUDALGORITHM_TRANSFORM_H
#define POINTCLOUDALGORITHM_TRANSFORM_H

/// @file Transform.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 刚体变换点坐标：p' = T * p（列向量语义）

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
/** 原地变换 xyz（3*N float，mm） */
POINT_CLOUD_ALGORITHM_API void transformXyzInPlace(std::vector<float>& xyz, const Eigen::Isometry3d& t);

/**
 * 变换拷贝到 outXyz
 * @param pointCount 点数；src 须至少 3*pointCount
 */
POINT_CLOUD_ALGORITHM_API void transformXyz(const float* srcXyz, std::size_t pointCount, const Eigen::Isometry3d& t,
											std::vector<float>& outXyz);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_TRANSFORM_H
