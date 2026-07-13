#pragma once

#include "MeshSurfaceReconstructionAmrtoTypes.h"

namespace geoalgo
{
namespace meshrecon
{

bool remeshToQuadMesh(
	const IndexedMeshLite& triIn,
	QuadMeshLite& quadOut,
	const InstantMeshesParams& params,
	std::string* errMsg);

bool writeQuadMeshObj(const QuadMeshLite& quad, const std::string& objPath, std::string* errMsg);

bool writeIndexedMeshObj(const IndexedMeshLite& mesh, const std::string& objPath, std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo
