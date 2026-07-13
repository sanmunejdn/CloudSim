#pragma once

#include "MeshSurfaceReconstructionInternal.h"
#include "MeshSurfaceReconstructionAmrtoTypes.h"

namespace geoalgo
{
namespace meshrecon
{

bool partitionQuadDomainsAmrtoImGmcg(
	const IndexedMeshLite& mesh,
	const MeshSurfaceReconstructParams& params,
	std::vector<QuadPatch>& patches,
	int& outJunctionCount,
	MeshSurfaceReconstructReport* partitionStats,
	std::string* errMsg,
	QuadMeshLite* outCachedQuadMesh = nullptr,
	GmcgResult* outCachedGmcg = nullptr);

} // namespace meshrecon
} // namespace geoalgo
