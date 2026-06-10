#pragma once

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{

struct VCg_ALGORITHMS_API RepairParams
{
	bool removeDegenerate = true;   // 去除退化面（面积≈0）
	bool removeDuplicate = true;    // 去除重复顶点
	bool removeNonManifold = true;  // 去除非流形边/顶点
	bool fillHoles = false;         // 填充孔洞（较慢）
	int holeMaxEdgeCount = 30;      // 孔洞最大边数
};

// 网格修复：去重 → 去退化 → 去非流形 → 填孔
VCg_ALGORITHMS_API bool repairMesh(
	const std::vector<float>& triangleSoup,
	std::vector<float>& outSoup,
	const RepairParams& params = {},
	std::string* errMsg = nullptr);

} // namespace vcgalgo
