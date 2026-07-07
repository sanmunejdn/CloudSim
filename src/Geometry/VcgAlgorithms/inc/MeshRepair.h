#pragma once

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
	bool fillHoles = false;
	int holeMaxEdgeCount = 30;
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

VCg_ALGORITHMS_API bool repairMesh(
	const std::vector<float>& triangleSoup,
	std::vector<float>& outSoup,
	const RepairParams& params = {},
	RepairReport* report = nullptr,
	std::string* errMsg = nullptr);

} // namespace vcgalgo
