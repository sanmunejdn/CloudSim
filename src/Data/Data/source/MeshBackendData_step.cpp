/// @file MeshBackendData_step.cpp
/// @brief Mesh STEP 单文件 tessellate（层级 STEP 走 BrepBackendData）

#include "pch.h"

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
