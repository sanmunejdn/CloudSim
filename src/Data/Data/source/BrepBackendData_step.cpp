/// @file BrepBackendData_step.cpp
/// @brief BrepBackendData_step 实现

#include "pch.h"

#include "BrepBackendData.h"
#include "RunLogger.h"

#include <Discretize.h>
#include <ShapeHandle.h>
#include <ShapeIo.h>

bool BrepBackendData::loadFromStepFile(const std::string& path, std::string* errMsg)
{
	geoalgo::ShapeHandle shape;
	if (!geoalgo::readStepIntoHandle(path, shape, errMsg))
	{
		return false;
	}
	setShape(std::move(shape));
	return true;
}

bool BrepBackendData::loadStepHierarchyFromFile(const std::string& path, std::vector<BrepHierarchyPart>& outParts,
												std::string* errMsg)
{
	outParts.clear();
	geoalgo::ShapeHandle assembly;
	if (!geoalgo::readStepIntoHandle(path, assembly, errMsg))
	{
		return false;
	}
	std::vector<geoalgo::MeshHierarchyPart> meshParts;
	if (!geoalgo::collectShapeHierarchyTopology(assembly, meshParts, errMsg))
	{
		return false;
	}
	if (meshParts.empty())
	{
		BrepHierarchyPart root;
		root.partPath = "0";
		root.parentPartPath.clear();
		root.displayName = "Solid_0";
		root.shapeRef = assembly;
		outParts.push_back(std::move(root));
	}
	else
	{
		outParts.reserve(meshParts.size());
		for (const geoalgo::MeshHierarchyPart& mp : meshParts)
		{
			BrepHierarchyPart bp;
			bp.partPath = mp.partPath;
			bp.parentPartPath = mp.parentPartPath;
			bp.displayName = mp.displayName;
			bp.shapeRef = assembly;
			outParts.push_back(std::move(bp));
		}
	}
	RunLogger::info("[BrepBackendData] STEP hierarchy loaded (shared assembly shape).");
	return true;
}
