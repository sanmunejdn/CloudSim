/// @file BrepBackendData_step.cpp
/// @brief Brep STEP 加载转发至 backend_io

#include "pch.h"

#include "BackendImporters.h"
#include "BrepBackendData.h"

bool BrepBackendData::loadFromStepFile(const std::string& path, std::string* errMsg)
{
	return backend_io::loadBrepFromStepFile(*this, path, errMsg);
}

bool BrepBackendData::loadStepHierarchyFromFile(const std::string& path, std::vector<BrepHierarchyPart>& outParts,
												std::string* errMsg, geoalgo::ShapeHandle* outAssembly)
{
	return backend_io::loadBrepStepHierarchy(path, outParts, errMsg, outAssembly);
}
