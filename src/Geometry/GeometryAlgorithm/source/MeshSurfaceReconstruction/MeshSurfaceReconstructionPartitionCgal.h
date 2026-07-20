#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONPARTITIONCGAL_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONPARTITIONCGAL_H

/// @file MeshSurfaceReconstructionPartitionCgal.h
/// @brief CGAL SDF 分割：每段取代表面索引，供 chart 种子融合

#include "MeshSurfaceReconstructionInternal.h"

#include <string>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
/// CGAL SDF 分割：每段取代表面索引，供 chart 种子融合
bool collectSdfSegmentSeedFaces(const IndexedMeshLite& mesh, int segmentCount, std::vector<int>& outSeedFaceIndices,
								std::string* errMsg = nullptr);

bool partitionQuadDomainsCgalChartHybrid(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
										 std::vector<QuadPatch>& patches, int& outJunctionCount,
										 MeshSurfaceReconstructReport* partitionStats, std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONPARTITIONCGAL_H
