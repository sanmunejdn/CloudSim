#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGQUADGRAPH_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGQUADGRAPH_H

/// @file MeshSurfaceReconstructionGmcgQuadGraph.h
/// @brief MeshSurfaceReconstructionGmcgQuadGraph 接口

#include "MeshSurfaceReconstructionAmrtoTypes.h"

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
struct GmcgHalfEdge
{
	int face = -1;
	int edgeInFace = 0;
	int from = -1;
	int to = -1;
	int twin = -1;
	int next = -1;
};

// quad 半边网格，供 motorcycle 与 chart 提取共用
struct GmcgQuadGraph
{
	std::vector<GmcgHalfEdge> halfEdges;
	std::vector<int> faceToHalfEdge;
	std::vector<std::vector<int>> vertexHalfEdges;
	int quadCount = 0;

	static GmcgQuadGraph build(const QuadMeshLite& mesh);

	int advanceMotorcycle(const int halfEdge) const;
	bool isBoundaryHalfEdge(const int halfEdge) const;
};

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGQUADGRAPH_H
