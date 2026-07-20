#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGUVPACK_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGUVPACK_H

/// @file MeshSurfaceReconstructionGmcgUvPack.h
/// @brief MeshSurfaceReconstructionGmcgUvPack 接口

#include "MeshSurfaceReconstructionAmrtoTypes.h"
#include "MeshSurfaceReconstructionGmcgChartExtract.h"
#include "MeshSurfaceReconstructionGmcgQuadGraph.h"

namespace geoalgo
{
namespace meshrecon
{
// 为每个 chart 生成 [0,1]^2 参数域 UV 与四角顶点
void packChartUvCoordinates(const QuadMeshLite& globalQuad, const GmcgQuadGraph& graph,
							const GmcgChartFaceGroups& groups, std::vector<QuadMeshLite>& chartMeshes,
							std::vector<std::vector<float>>& chartVertexUv);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGUVPACK_H
