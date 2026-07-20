#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONAMRTOPARTITION_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONAMRTOPARTITION_H

/// @file MeshSurfaceReconstructionAmrtoPartition.h
/// @brief MeshSurfaceReconstructionAmrtoPartition 接口

#include "MeshSurfaceReconstructionAmrtoTypes.h"
#include "MeshSurfaceReconstructionInternal.h"

namespace geoalgo
{
namespace meshrecon
{
bool partitionQuadDomainsAmrtoImGmcg(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
									 std::vector<QuadPatch>& patches, int& outJunctionCount,
									 MeshSurfaceReconstructReport* partitionStats, std::string* errMsg,
									 QuadMeshLite* outCachedQuadMesh = nullptr, GmcgResult* outCachedGmcg = nullptr);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONAMRTOPARTITION_H
