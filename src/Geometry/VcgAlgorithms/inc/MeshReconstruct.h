#ifndef VCGALGORITHMS_MESHRECONSTRUCT_H
#define VCGALGORITHMS_MESHRECONSTRUCT_H

/// @file MeshReconstruct.h
/// @brief 重建后处理管线：简化 → 修复 → 可选平滑（输入已是 soup）

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{
/**
 * 对已有三角 soup 做 vcglib 后处理（不调用 Poisson）
 * Host/Data 侧「重建+后处理」通常先 pclalgo::reconstructPoisson* 再调本函数
 * @param triangleSoup 9*T float，mm
 * @param targetFaceCount >0 时简化到该面数；0=不简化
 * @param doRepair 默认 true
 * @param doSmooth 默认 false（Laplacian 2 次）
 * @return false：soup 非法或最终无面
 */
VCg_ALGORITHMS_API bool postProcessReconstructedMesh(const std::vector<float>& triangleSoup,
													 std::vector<float>& outSoup, int targetFaceCount = 0,
													 bool doRepair = true, bool doSmooth = false,
													 std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHRECONSTRUCT_H
