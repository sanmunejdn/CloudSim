#ifndef VCGALGORITHMS_MESHREMESH_H
#define VCGALGORITHMS_MESHREMESH_H

/// @file MeshRemesh.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 各向同性重网格与边长中位数统计

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{
/**
 * 统计三角 soup 边长中位数（mm）
 * @return false：soup 非法或无边
 */
VCg_ALGORITHMS_API bool computeMedianEdgeLengthMm(const std::vector<float>& triangleSoup, double& outMedianMm,
												  std::string* errMsg = nullptr);

/**
 * vcglib IsotropicRemeshing：均匀化三角形分布
 * @param targetEdgeLengthMm 目标边长 mm；须 >0
 * @param iterations 优化迭代，默认 3
 * @param featureAngleDeg 特征边保护角 °，默认 30
 * @return false：soup 非法、目标边长无效或重网格失败
 */
VCg_ALGORITHMS_API bool isotropicRemesh(const std::vector<float>& triangleSoup, double targetEdgeLengthMm,
										std::vector<float>& outSoup, int iterations = 3, double featureAngleDeg = 30.0,
										std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHREMESH_H
