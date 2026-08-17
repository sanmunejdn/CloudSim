/// @file MeshBackendData_step.cpp
/// @brief Mesh 后端数据

#include "pch.h"

#include "MeshBackendData.h"
#include "MeshBackendData_loaders.h"
#include "RunLogger.h"

#include <Discretize.h>
#include <Types.h>

namespace mesh_backend_load
{
bool meshLoadStepSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg)
{
	geoalgo::TessellateParams params;
	params.flipReversedFaces = kMeshStepFlipReversedFaceWinding;
	return geoalgo::tessellateStepFile(path, params, soup, errMsg);
}

} // namespace mesh_backend_load

bool MeshBackendData::loadStepHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts,
												std::string* errMsg)
{
	outParts.clear();
	std::vector<geoalgo::MeshHierarchyPart> parts;
	geoalgo::TessellateParams params;
	params.flipReversedFaces = mesh_backend_load::kMeshStepFlipReversedFaceWinding;
	if (!geoalgo::tessellateStepHierarchy(path, params, parts, errMsg))
	{
		return false;
	}
	outParts.reserve(parts.size());
	for (const geoalgo::MeshHierarchyPart& p : parts)
	{
		MeshHierarchyPart mp;
		mp.partPath = p.partPath;
		mp.parentPartPath = p.parentPartPath;
		mp.displayName = p.displayName;
		mp.triangleSoup = p.triangleSoup;
		outParts.push_back(std::move(mp));
	}
	RunLogger::info("[MeshBackendData] STEP hierarchy loaded successfully.");
	return true;
}
