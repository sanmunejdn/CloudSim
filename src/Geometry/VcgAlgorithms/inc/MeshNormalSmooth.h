#pragma once

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{

/// 论文 Ch2：法矢 Kuwahara + 拉普拉斯光顺参数
struct MeshNormalSmoothParams
{
	int iterations = 6;
	double featureThresholdC0 = 0.8;
	double laplacianLambda = 0.5;
	double bilateralK = 2.0;
};

/**
 * 基于三角片法矢调整的网格光顺，输出新 triangle soup（mm）
 * @param outGapVolume 可选，光顺前后近似间隙体积
 */
VCg_ALGORITHMS_API bool smoothMeshByNormalAdjustment(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	const MeshNormalSmoothParams& params,
	double* outGapVolume = nullptr,
	std::string* errMsg = nullptr);

} // namespace vcgalgo
