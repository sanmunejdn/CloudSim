#pragma once

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{

// Laplacian 平滑：快速，适合轻度噪声
VCg_ALGORITHMS_API bool smoothLaplacian(
	const std::vector<float>& triangleSoup,
	int iterations,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

// Implicit Fairing 平滑：保形，适合重度噪声
// lambda 越大平滑越强（典型 0.1 ~ 0.5）
VCg_ALGORITHMS_API bool smoothImplicitFairing(
	const std::vector<float>& triangleSoup,
	double lambda,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

} // namespace vcgalgo
