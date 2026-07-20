#ifndef POINTCLOUDALGORITHM_RECONSTRUCTIONCONFIG_H
#define POINTCLOUDALGORITHM_RECONSTRUCTIONCONFIG_H

/// @file ReconstructionConfig.h
/// @brief ReconstructionConfig 接口

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{
enum class ReconstructionQuality
{
	Fast,	  // 快速模式: 更大体素、更少迭代
	Balanced, // 平衡模式: 默认参数
	Quality	  // 质量模式: 更小体素、更多迭代
};

struct POINT_CLOUD_ALGORITHM_API ReconstructionConfig
{
	ReconstructionQuality quality = ReconstructionQuality::Balanced;
	double maxPointsForReconstruction = 500000; // 最大点数限制
	bool enableParallel = true;					// 是否启用并行

	// 根据质量级别获取参数
	double getVoxelPrefilterMm() const;
	double getSpacingMm() const;
	int getSmoothIterations() const;
	double getOutlierRemovalPercent() const;
};

// 新增配置版本API
POINT_CLOUD_ALGORITHM_API bool reconstructPoissonWithConfig(std::vector<float> xyz, std::vector<float> normals,
															std::vector<float>& triangleSoupOut,
															const ReconstructionConfig& config,
															std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool reconstructPoissonAutoWithConfig(std::vector<float> xyz,
																std::vector<float>& triangleSoupOut,
																const ReconstructionConfig& config,
																std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool reconstructScaleSpaceWithConfig(std::vector<float> xyz,
															   std::vector<float>& triangleSoupOut,
															   const ReconstructionConfig& config,
															   std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_RECONSTRUCTIONCONFIG_H
