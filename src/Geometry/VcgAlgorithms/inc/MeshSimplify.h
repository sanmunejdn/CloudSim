#ifndef VCGALGORITHMS_MESHSIMPLIFY_H
#define VCGALGORITHMS_MESHSIMPLIFY_H

/// @file MeshSimplify.h
/// @brief 网格简化：quadric-error 边折叠

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{
struct VCg_ALGORITHMS_API SimplifyParams
{
	int targetFaceCount = 0;	   ///< 目标面数；0=约保留原面数一半
	double qualityThreshold = 0.3; ///< 0–1，越大质量越高、越慢
	bool preserveBoundary = true;  ///< 保留边界边
	bool preserveTopology = true;  ///< 保持拓扑
	int optimizePriority = 0;	   ///< 0=质量优先，1=速度优先
};

/**
 * vcglib QuadricEdgeCollapse；输入/输出均为 9*T float soup（mm）
 * @return false：soup 非法、目标面数异常或简化失败
 */
VCg_ALGORITHMS_API bool simplifyQuadricEdgeCollapse(const std::vector<float>& triangleSoup, std::vector<float>& outSoup,
													const SimplifyParams& params = {}, std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHSIMPLIFY_H
