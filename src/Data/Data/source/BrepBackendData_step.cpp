/// @file BrepBackendData_step.cpp
/// @brief Brep 后端数据

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
												std::string* errMsg, geoalgo::ShapeHandle* outAssembly)
{
	outParts.clear();
	geoalgo::ShapeHandle assembly;
	if (!geoalgo::readStepIntoHandle(path, assembly, errMsg))
	{
		return false;
	}
	if (outAssembly)
	{
		*outAssembly = assembly;
	}
	std::vector<geoalgo::ShapeHierarchyPart> solids;
	if (!geoalgo::collectBrepSolidParts(assembly, solids, errMsg))
	{
		return false;
	}
	outParts.reserve(solids.size());
	for (const geoalgo::ShapeHierarchyPart& sp : solids)
	{
		BrepHierarchyPart bp;
		bp.partPath = sp.partPath;
		bp.parentPartPath = sp.parentPartPath;
		bp.displayName = sp.displayName;
		bp.shapeRef = sp.shape;
		outParts.push_back(std::move(bp));
	}
	RunLogger::info("[BrepBackendData] STEP hierarchy loaded (per-solid shapes).");
	return true;
}
