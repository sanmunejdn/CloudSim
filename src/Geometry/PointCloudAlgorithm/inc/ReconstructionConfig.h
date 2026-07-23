#ifndef POINTCLOUDALGORITHM_RECONSTRUCTIONCONFIG_H
#define POINTCLOUDALGORITHM_RECONSTRUCTIONCONFIG_H

/// @file ReconstructionConfig.h
/// @brief 重建质量档位与带配置入口（Poisson / Scale-space）

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{
/// 质量档：影响体素预滤波、离群%、Scale-space 平滑迭代
enum class ReconstructionQuality
{
	Fast,	  ///< 体素 2.0mm / 平滑 2 / 离群 3%
	Balanced, ///< 体素 1.0mm / 平滑 4 / 离群 5%
	Quality	  ///< 体素 0.5mm / 平滑 6 / 离群 7%
};

struct POINT_CLOUD_ALGORITHM_API ReconstructionConfig
{
	ReconstructionQuality quality = ReconstructionQuality::Balanced;
	double maxPointsForReconstruction = 500000; ///< 超限则自动体素下采样
	bool enableParallel = true;					///< TBB 可用时并行

	double getVoxelPrefilterMm() const;
	double getSpacingMm() const;
	int getSmoothIterations() const;
	double getOutlierRemovalPercent() const;
};

/**
 * 调用方已提供法线；超 maxPoints 时体素下采样并重算 PCA 法线后 Poisson
 * @return false：点数不足、下采样/法线/Poisson 失败
 */
POINT_CLOUD_ALGORITHM_API bool reconstructPoissonWithConfig(std::vector<float> xyz, std::vector<float> normals,
															std::vector<float>& triangleSoupOut,
															const ReconstructionConfig& config,
															std::string* errMsg = nullptr);

/**
 * 完整预处理 + Poisson（推荐默认路径）
 * @return false：预处理或 Poisson 失败
 */
POINT_CLOUD_ALGORITHM_API bool reconstructPoissonAutoWithConfig(std::vector<float> xyz,
																std::vector<float>& triangleSoupOut,
																const ReconstructionConfig& config,
																std::string* errMsg = nullptr);

/**
 * Scale-space；超点数体素下采样；smoothIterations 取自质量档
 * @return false：输入非法或无三角面
 */
POINT_CLOUD_ALGORITHM_API bool reconstructScaleSpaceWithConfig(std::vector<float> xyz,
															   std::vector<float>& triangleSoupOut,
															   const ReconstructionConfig& config,
															   std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_RECONSTRUCTIONCONFIG_H
