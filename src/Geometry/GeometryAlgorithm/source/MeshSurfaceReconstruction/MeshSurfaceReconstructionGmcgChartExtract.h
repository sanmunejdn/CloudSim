#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGCHARTEXTRACT_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGCHARTEXTRACT_H

/// @file MeshSurfaceReconstructionGmcgChartExtract.h
/// @brief MeshSurfaceReconstructionGmcgChartExtract 接口

#include "MeshSurfaceReconstructionGmcgMotorcycleTrace.h"
#include "MeshSurfaceReconstructionGmcgQuadGraph.h"

#include <vector>

namespace geoalgo
{
namespace meshrecon
{
struct GmcgChartFaceGroups
{
	std::vector<std::vector<int>> chartFaces;
};

GmcgChartFaceGroups extractCharts(const GmcgQuadGraph& graph, const GmcgMotorcycleResult& traceResult);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGCHARTEXTRACT_H
