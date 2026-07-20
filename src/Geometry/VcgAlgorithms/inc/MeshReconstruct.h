#ifndef VCGALGORITHMS_MESHRECONSTRUCT_H
#define VCGALGORITHMS_MESHRECONSTRUCT_H

/// @file MeshReconstruct.h
/// @brief MeshReconstruct 接口

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo

{
// CGAL Poisson 重建后的 vcglib 后处理管线（输入为 triangle soup，9 floats/面）

// targetFaceCount > 0 时执行简化；doRepair 执行修复；doSmooth 执行平滑

VCg_ALGORITHMS_API bool postProcessReconstructedMesh(

	const std::vector<float>& triangleSoup,

	std::vector<float>& outSoup,

	int targetFaceCount = 0,

	bool doRepair = true,

	bool doSmooth = false,

	std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHRECONSTRUCT_H
