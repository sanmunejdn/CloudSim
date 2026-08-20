#ifndef POINTCLOUDALGORITHM_POINTFEATURES_H
#define POINTCLOUDALGORITHM_POINTFEATURES_H

/// @file PointFeatures.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief SPFH / FPFH 局部特征（供 RANSAC 全局配准）

#include "point_cloud_algorithm_global.h"

#include <vector>

namespace pclalgo
{
constexpr std::size_t kFpfhDim = 33U; ///< 单点 FPFH 维度

/**
 * 计算每点 SPFH
 * @param outSpfh 长度 N*33；xyz/normals 须 3*N 且同序
 */
POINT_CLOUD_ALGORITHM_API void computeSpfhForCloud(const std::vector<float>& xyz, const std::vector<float>& normals,
												   const unsigned int kNeighbors, std::vector<float>& outSpfh);

/**
 * 由 SPFH 聚合 FPFH
 * @param spfh 须已由 computeSpfhForCloud 得到
 */
POINT_CLOUD_ALGORITHM_API void computeFpfhForCloud(const std::vector<float>& xyz, const std::vector<float>& normals,
												   const std::vector<float>& spfh, const unsigned int kNeighbors,
												   std::vector<float>& outFpfh);

/** 两 FPFH 描述子 L2 距离；a/b 各指向 33 float */
POINT_CLOUD_ALGORITHM_API float fpfhL2Distance(const float* a, const float* b);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_POINTFEATURES_H
