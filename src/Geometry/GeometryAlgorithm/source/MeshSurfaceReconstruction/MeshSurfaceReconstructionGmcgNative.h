#pragma once

#include "MeshSurfaceReconstructionAmrtoTypes.h"

namespace geoalgo
{
namespace meshrecon
{

bool partitionQuadMeshNativeGmcg(
	const QuadMeshLite& quadMesh,
	GmcgResult& outResult,
	std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo
