#ifndef VCGALGORITHMS_MESHREPAIR_H
#define VCGALGORITHMS_MESHREPAIR_H

/// @file MeshRepair.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 网格修复：去退化/重复/非流形、可选填孔

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{
struct VCg_ALGORITHMS_API RepairParams
{
	bool removeDegenerate = true;
	bool removeDuplicate = true;
	bool removeDuplicateFaces = true;
	bool removeNonManifold = true;
	bool fillHoles = false;		 ///< 填孔默认关（避免误封开口）
	int holeMaxEdgeCount = 30; ///< 可填孔的最大边界边数
};

struct VCg_ALGORITHMS_API RepairReport
{
	int inputFaceCount = 0;
	int outputFaceCount = 0;
	int removedDuplicateFaces = 0;
	int removedDegenerateFaces = 0;
	int removedNonManifoldFaces = 0;
	int facesAddedByFill = 0;
};

/**
 * vcglib Clean + 可选 Hole；soup 9*T float（mm）
 * @return false：输入非法或修复后无面
 */
VCg_ALGORITHMS_API bool repairMesh(const std::vector<float>& triangleSoup, std::vector<float>& outSoup,
								   const RepairParams& params = {}, RepairReport* report = nullptr,
								   std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHREPAIR_H
