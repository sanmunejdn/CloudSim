/// @file MeshReconstruct.cpp
/// @brief MeshReconstruct 实现

#include "MeshReconstruct.h"

#include "MeshRepair.h"
#include "MeshSimplify.h"
#include "MeshSmooth.h"

namespace vcgalgo
{
bool postProcessReconstructedMesh(const std::vector<float>& triangleSoup, std::vector<float>& outSoup,
								  int targetFaceCount, bool doRepair, bool doSmooth, std::string* errMsg)
{
	outSoup.clear();

	if (triangleSoup.empty() || triangleSoup.size() % 9 != 0)
	{
		if (errMsg)
			*errMsg = "postProcessReconstructedMesh: invalid triangle soup";
		return false;
	}

	std::vector<float> current = triangleSoup;

	if (targetFaceCount > 0)
	{
		std::vector<float> simplified;
		SimplifyParams sParams;
		sParams.targetFaceCount = targetFaceCount;
		sParams.preserveBoundary = true;
		sParams.preserveTopology = true;
		if (simplifyQuadricEdgeCollapse(current, simplified, sParams, errMsg))
		{
			current = std::move(simplified);
		}
	}

	if (doRepair)
	{
		std::vector<float> repaired;
		RepairParams rParams;
		rParams.removeDegenerate = true;
		rParams.removeDuplicate = true;
		rParams.removeDuplicateFaces = true;
		rParams.removeNonManifold = true;
		rParams.fillHoles = false;
		if (repairMesh(current, repaired, rParams, nullptr, errMsg))
		{
			current = std::move(repaired);
		}
	}

	if (doSmooth)
	{
		std::vector<float> smoothed;
		if (smoothLaplacian(current, 2, smoothed, errMsg))
		{
			current = std::move(smoothed);
		}
	}

	outSoup = std::move(current);
	return !outSoup.empty();
}

} // namespace vcgalgo
