#pragma once

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{

// 各向同性重网格：均匀化三角形分布
// targetEdgeLengthMm：目标边长（mm）
// iterations：优化迭代次数（默认 3）
VCg_ALGORITHMS_API bool isotropicRemesh(
	const std::vector<float>& triangleSoup,
	double targetEdgeLengthMm,
	std::vector<float>& outSoup,
	int iterations = 3,
	std::string* errMsg = nullptr);

} // namespace vcgalgo
