#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGNATIVE_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGNATIVE_H

/// @file MeshSurfaceReconstructionGmcgNative.h
/// @brief MeshSurfaceReconstructionGmcgNative 接口

#include "MeshSurfaceReconstructionAmrtoTypes.h"

namespace geoalgo
{
namespace meshrecon
{
bool partitionQuadMeshNativeGmcg(const QuadMeshLite& quadMesh, GmcgResult& outResult, std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGNATIVE_H
