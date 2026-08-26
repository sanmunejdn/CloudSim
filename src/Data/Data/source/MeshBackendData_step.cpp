/// @file MeshBackendData_step.cpp
/// @brief Mesh STEP 辅助与层级加载转发

#include "pch.h"

#include "BackendImporters.h"
#include "MeshBackendData.h"
#include "MeshBackendData_loaders.h"

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
	return backend_io::loadMeshStepHierarchy(path, outParts, errMsg);
}
