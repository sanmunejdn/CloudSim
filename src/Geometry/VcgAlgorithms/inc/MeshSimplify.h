#pragma once

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{

struct VCg_ALGORITHMS_API SimplifyParams
{
	int targetFaceCount = 0;           // 目标面数，0=保留原面数一半
	double qualityThreshold = 0.3;     // 质量阈值 0-1，越大越慢但质量越高
	bool preserveBoundary = true;      // 是否保留边界边
	bool preserveTopology = true;      // 是否保持拓扑
	int optimizePriority = 0;          // 0=质量优先，1=速度优先
};

// quadric-error edge collapse 简化
VCg_ALGORITHMS_API bool simplifyQuadricEdgeCollapse(
	const std::vector<float>& triangleSoup,
	std::vector<float>& outSoup,
	const SimplifyParams& params = {},
	std::string* errMsg = nullptr);

} // namespace vcgalgo
