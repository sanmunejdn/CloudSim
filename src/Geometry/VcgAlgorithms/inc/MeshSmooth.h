#ifndef VCGALGORITHMS_MESHSMOOTH_H
#define VCGALGORITHMS_MESHSMOOTH_H

/// @file MeshSmooth.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 网格平滑：Laplacian / Taubin（可选平滑前修复）

#include "vcg_algorithms_global.h"

#include "MeshRepair.h"

#include <string>
#include <vector>

namespace vcgalgo
{
struct VCg_ALGORITHMS_API MeshSmoothParams
{
	int iterations = 3;				 ///< 迭代次数
	double lambda = 0.2;			 ///< 步长；Taubin 时作 λ
	bool useTaubin = false;			 ///< true=Taubin λ/μ 保形
	bool preserveBoundary = true;	 ///< 边界保护
	bool cotangentWeight = true;	 ///< 余切权重（Laplacian）
	bool repairBeforeSmooth = false; ///< 平滑前先 repairMesh
	RepairParams repairParams{};
};

/**
 * 统一入口：按 params 选 Laplacian 或 Taubin；可选先修复
 * @param repairReport 仅 repairBeforeSmooth 时有意义
 * @return false：soup 非法或平滑失败
 */
VCg_ALGORITHMS_API bool applyMeshSmooth(const std::vector<float>& triangleSoup, std::vector<float>& outSoup,
										const MeshSmoothParams& params, RepairReport* repairReport = nullptr,
										std::string* errMsg = nullptr);

/**
 * Laplacian 顶点坐标平滑（快速）
 * @param iterations 默认调用方可传 3
 */
VCg_ALGORITHMS_API bool smoothLaplacian(const std::vector<float>& triangleSoup, int iterations,
										std::vector<float>& outSoup, std::string* errMsg = nullptr);

/**
 * Taubin λ/μ 保形平滑；μ 由实现取 −lambda
 * @param lambda 默认常用 0.2
 */
VCg_ALGORITHMS_API bool smoothTaubin(const std::vector<float>& triangleSoup, int iterations, double lambda,
									 std::vector<float>& outSoup, std::string* errMsg = nullptr);

/** 兼容旧名：固定 3 次 Taubin */
inline bool smoothImplicitFairing(const std::vector<float>& triangleSoup, double lambda, std::vector<float>& outSoup,
								  std::string* errMsg = nullptr)
{
	return smoothTaubin(triangleSoup, 3, lambda, outSoup, errMsg);
}

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHSMOOTH_H
