#ifndef POINTCLOUDALGORITHM_PREPROCESS_H
#define POINTCLOUDALGORITHM_PREPROCESS_H

/// @file Preprocess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 点云预处理：法线估计/定向、离群剔除、双边平滑、重建前管线

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{
/**
 * PCA 估计法线（CGAL pca_estimate_normals）
 * @param xyz 3*N float，mm
 * @param normalsOut 3*N，与 xyz 同序
 * @param kNeighbors 邻域点数，默认 12
 * @return false：点数不足或 CGAL 失败
 */
POINT_CLOUD_ALGORITHM_API bool estimateNormalsPca(const std::vector<float>& xyz, std::vector<float>& normalsOut,
												  unsigned int kNeighbors = 12, std::string* errMsg = nullptr);

/**
 * Jet 拟合估计法线（更高阶，噪声下更稳、更慢）
 * @param degreeFitting Jet 阶数，默认 2
 */
POINT_CLOUD_ALGORITHM_API bool estimateNormalsJet(const std::vector<float>& xyz, std::vector<float>& normalsOut,
												  unsigned int kNeighbors = 12, unsigned int degreeFitting = 2,
												  std::string* errMsg = nullptr);

/**
 * MST 传播统一法线朝向（原地改 normals；可选同步 rgba）
 * @return false：点数/法线长度不符或定向失败
 */
POINT_CLOUD_ALGORITHM_API bool orientNormalsMst(std::vector<float>& xyz, std::vector<float>& normalsInOut,
												unsigned int kNeighbors = 12, std::vector<float>* rgbaInOut = nullptr,
												std::string* errMsg = nullptr);

/**
 * 统计离群剔除（CGAL remove_outliers），按距离百分位删点
 * @param removalPercent 剔除比例 0–100，默认 5
 * @param kNeighbors 邻域，默认 24；可选同步删 normals/rgba
 */
POINT_CLOUD_ALGORITHM_API bool removeOutliers(std::vector<float>& xyzInOut, double removalPercent = 5.0,
											  unsigned int kNeighbors = 24, std::vector<float>* normalsInOut = nullptr,
											  std::vector<float>* rgbaInOut = nullptr, std::string* errMsg = nullptr);

/**
 * 双边平滑点集（需已有法线时效果更好；可同步改 normals）
 * @return false：输入非法或平滑失败
 */
POINT_CLOUD_ALGORITHM_API bool smoothBilateral(std::vector<float>& xyzInOut, std::vector<float>* normalsInOut = nullptr,
											   std::string* errMsg = nullptr);

/**
 * 重建前管线：可选体素 → 离群(k=24) → PCA 法线(k=12) → MST 定向(k=12)
 * @param voxelPrefilterMm 体素边长 mm；≤0 跳过
 * @param outlierRemovalPercent 离群%；≤0 跳过
 * @return false：任一步失败或最终点数不足
 */
POINT_CLOUD_ALGORITHM_API bool preprocessForReconstruction(std::vector<float>& xyzInOut, std::vector<float>& normalsOut,
														   double voxelPrefilterMm, double outlierRemovalPercent,
														   std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_PREPROCESS_H
