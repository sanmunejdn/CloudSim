#include "MeshSurfaceReconstructionGmcgNative.h"
#include "MeshSurfaceReconstructionAmrtoLoader.h"
#include "MeshSurfaceReconstructionGmcgQuadGraph.h"
#include "MeshSurfaceReconstructionGmcgMotorcycleTrace.h"
#include "MeshSurfaceReconstructionGmcgChartExtract.h"
#include "MeshSurfaceReconstructionGmcgUvPack.h"

#include "RunLogger.h"

namespace geoalgo
{
namespace meshrecon
{
namespace
{

bool buildGmcgResultFromPipeline(const QuadMeshLite& quadMesh, GmcgResult& outResult)
{
	const GmcgQuadGraph graph = GmcgQuadGraph::build(quadMesh);
	if (graph.quadCount < 1)
	{
		return false;
	}
	const GmcgMotorcycleResult trace = traceMotorcycles(graph);
	const GmcgChartFaceGroups groups = extractCharts(graph, trace);
	if (groups.chartFaces.empty())
	{
		return false;
	}

	std::vector<QuadMeshLite> chartMeshes;
	std::vector<std::vector<float>> chartUvs;
	packChartUvCoordinates(quadMesh, graph, groups, chartMeshes, chartUvs);

	outResult.globalQuad = quadMesh;
	outResult.charts.clear();
	outResult.charts.reserve(chartMeshes.size());
	for (std::size_t ci = 0; ci < chartMeshes.size(); ++ci)
	{
		GmcgChart chart;
		chart.chartId = static_cast<int>(ci);
		chart.quadMesh = std::move(chartMeshes[ci]);
		chart.vertexUv = chart.quadMesh.vertexUv;
		detectChartCornersFromUv(chart.quadMesh, chart.cornerVertexIndices);
		outResult.charts.push_back(std::move(chart));
	}
	return !outResult.charts.empty();
}

} // namespace

bool partitionQuadMeshNativeGmcg(
	const QuadMeshLite& quadMesh,
	GmcgResult& outResult,
	std::string* errMsg)
{
	outResult = {};
	const int qCount = static_cast<int>(quadMesh.quadFaces.size() / 4U);
	if (qCount < 1)
	{
		if (errMsg)
		{
			*errMsg = "native gmcg: empty quad mesh";
		}
		return false;
	}

	if (!buildGmcgResultFromPipeline(quadMesh, outResult))
	{
		if (errMsg)
		{
			*errMsg = "native gmcg motorcycle pipeline failed";
		}
		return false;
	}

	RunLogger::info(
		std::string("native gmcg (motorcycle): ") + std::to_string(outResult.charts.size()) + " charts from "
		+ std::to_string(qCount) + " quads");
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
