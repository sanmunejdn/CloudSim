#ifndef POINTCLOUDALGORITHM_DOWNSAMPLE_H
#define POINTCLOUDALGORITHM_DOWNSAMPLE_H

/// @file Downsample.h
/// @brief 点云下采样：体素格 / 随机保留

#include "point_cloud_algorithm_global.h"

#include <vector>

namespace pclalgo
{
/**
 * CGAL 体素格下采样；每格保留代表点，可选同步 rgba
 * @param xyzInOut 3*N float，mm；原地改写
 * @param voxelSizeMm 体素边长 mm；≤0 时不改变点集
 * @param minPointsPerCell 格内最少点数门槛，默认 1
 * @return false：输入长度非法
 */
POINT_CLOUD_ALGORITHM_API bool downsampleVoxelGrid(std::vector<float>& xyzInOut, double voxelSizeMm,
												   unsigned int minPointsPerCell = 1,
												   std::vector<float>* rgbaInOut = nullptr);

/**
 * 随机下采样，按比例保留点（可选同步 rgba）
 * @param retainedFraction 保留比例 (0,1]；≤0 或 ≥1 时行为见实现（通常清空或全留）
 * @return false：输入长度非法
 */
POINT_CLOUD_ALGORITHM_API bool downsampleRandom(std::vector<float>& xyzInOut, double retainedFraction,
												std::vector<float>* rgbaInOut = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_DOWNSAMPLE_H
