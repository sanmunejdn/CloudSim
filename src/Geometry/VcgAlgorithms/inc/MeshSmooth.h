#ifndef VCGALGORITHMS_MESHSMOOTH_H
#define VCGALGORITHMS_MESHSMOOTH_H

/// @file MeshSmooth.h
/// @brief MeshSmooth 接口

#include "vcg_algorithms_global.h"

#include "MeshRepair.h"

#include <string>
#include <vector>

namespace vcgalgo
{
struct VCg_ALGORITHMS_API MeshSmoothParams
{
	int iterations = 3;
	double lambda = 0.2;
	bool useTaubin = false;
	bool preserveBoundary = true;
	bool cotangentWeight = true;
	bool repairBeforeSmooth = false;
	RepairParams repairParams{};
};

VCg_ALGORITHMS_API bool applyMeshSmooth(const std::vector<float>& triangleSoup, std::vector<float>& outSoup,
										const MeshSmoothParams& params, RepairReport* repairReport = nullptr,
										std::string* errMsg = nullptr);

VCg_ALGORITHMS_API bool smoothLaplacian(const std::vector<float>& triangleSoup, int iterations,
										std::vector<float>& outSoup, std::string* errMsg = nullptr);

// Taubin λ/μ 平滑；保留旧名供自检与外部兼容
VCg_ALGORITHMS_API bool smoothTaubin(const std::vector<float>& triangleSoup, int iterations, double lambda,
									 std::vector<float>& outSoup, std::string* errMsg = nullptr);

inline bool smoothImplicitFairing(const std::vector<float>& triangleSoup, double lambda, std::vector<float>& outSoup,
								  std::string* errMsg = nullptr)
{
	return smoothTaubin(triangleSoup, 3, lambda, outSoup, errMsg);
}

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHSMOOTH_H
