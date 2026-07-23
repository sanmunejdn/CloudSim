#ifndef VCGALGORITHMS_MESHNORMALSMOOTH_H
#define VCGALGORITHMS_MESHNORMALSMOOTH_H

/// @file MeshNormalSmooth.h
/// @brief 法矢 Kuwahara + 拉普拉斯光顺后回写顶点（论文 Ch2）

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{
struct MeshNormalSmoothParams
{
	int iterations = 6;				 ///< 外迭代
	double featureThresholdC0 = 0.8; ///< 特征保持阈值
	double laplacianLambda = 0.5;	 ///< 法矢拉普拉斯步长
	double bilateralK = 2.0;		 ///< 双边核尺度
};

/**
 * 基于三角片法矢调整的网格光顺，输出新 triangle soup（mm）
 * @param outGapVolume 可选，光顺前后近似间隙体积
 * @return false：soup 非法或光顺失败
 */
VCg_ALGORITHMS_API bool smoothMeshByNormalAdjustment(const std::vector<float>& soupIn, std::vector<float>& soupOut,
													 const MeshNormalSmoothParams& params,
													 double* outGapVolume = nullptr, std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHNORMALSMOOTH_H
