#pragma once

#include "MeshSurfaceReconstructionGmcgQuadGraph.h"

#include <vector>

namespace geoalgo
{
namespace meshrecon
{

struct GmcgMotorcycleResult
{
	std::vector<uint8_t> barrierHalfEdges;
	std::vector<int> vertexValence;
};

// 从奇异点发射 motorcycle，在半边网格上留下 barrier
GmcgMotorcycleResult traceMotorcycles(const GmcgQuadGraph& graph);

} // namespace meshrecon
} // namespace geoalgo
