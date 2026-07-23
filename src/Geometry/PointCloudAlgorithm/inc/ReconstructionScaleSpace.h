#ifndef POINTCLOUDALGORITHM_RECONSTRUCTIONSCALESPACE_H
#define POINTCLOUDALGORITHM_RECONSTRUCTIONSCALESPACE_H

/// @file ReconstructionScaleSpace.h
/// @brief Scale-space 重建：仅坐标点云 → 三角 soup（mm），不依赖法线

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{
/**
 * CGAL Scale_space_surface_reconstruction_3：多尺度平滑后提面
 * 对噪声更宽容；不跑离群/法线预处理；尖锐特征易钝化
 * @param xyz 3*N float，mm
 * @param triangleSoupOut 9*T float；成功后尝试焊点+绕序校正
 * @param smoothIterations increase_scale 次数，默认 4；越大越平滑、细节越少
 * @param meshingRadiusMm 历史参数；≤0 时按包围盒对角×0.05 计算，当前未传入 CGAL
 * @return false：xyz 非法，或无三角面（errMsg）
 */
POINT_CLOUD_ALGORITHM_API bool reconstructScaleSpace(const std::vector<float>& xyz, std::vector<float>& triangleSoupOut,
													 std::size_t smoothIterations = 4, double meshingRadiusMm = 0.0,
													 std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_RECONSTRUCTIONSCALESPACE_H
