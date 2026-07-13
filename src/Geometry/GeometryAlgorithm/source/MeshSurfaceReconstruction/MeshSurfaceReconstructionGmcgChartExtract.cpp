#include "MeshSurfaceReconstructionGmcgChartExtract.h"

#include <queue>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

bool sharesNonBarrierEdge(
	const GmcgQuadGraph& graph,
	const std::vector<uint8_t>& barriers,
	const int faceA,
	const int faceB)
{
	if (faceA < 0 || faceB < 0 || faceA == faceB)
	{
		return false;
	}
	const int baseA = faceA * 4;
	for (int e = 0; e < 4; ++e)
	{
		const int he = baseA + e;
		const int twin = graph.halfEdges[static_cast<std::size_t>(he)].twin;
		if (twin < 0)
		{
			continue;
		}
		if (graph.halfEdges[static_cast<std::size_t>(twin)].face != faceB)
		{
			continue;
		}
		if (barriers[static_cast<std::size_t>(he)] != 0U
			|| barriers[static_cast<std::size_t>(twin)] != 0U)
		{
			return false;
		}
		return true;
	}
	return false;
}

} // namespace

GmcgChartFaceGroups extractCharts(
	const GmcgQuadGraph& graph,
	const GmcgMotorcycleResult& traceResult)
{
	GmcgChartFaceGroups groups;
	const int qCount = graph.quadCount;
	if (qCount < 1)
	{
		return groups;
	}
	std::vector<int> faceChart(static_cast<std::size_t>(qCount), -1);
	int nextChart = 0;
	for (int seed = 0; seed < qCount; ++seed)
	{
		if (faceChart[static_cast<std::size_t>(seed)] >= 0)
		{
			continue;
		}
		const int chartId = nextChart++;
		std::queue<int> q;
		q.push(seed);
		faceChart[static_cast<std::size_t>(seed)] = chartId;
		while (!q.empty())
		{
			const int fi = q.front();
			q.pop();
			for (int ofi = 0; ofi < qCount; ++ofi)
			{
				if (faceChart[static_cast<std::size_t>(ofi)] >= 0)
				{
					continue;
				}
				if (!sharesNonBarrierEdge(graph, traceResult.barrierHalfEdges, fi, ofi))
				{
					continue;
				}
				faceChart[static_cast<std::size_t>(ofi)] = chartId;
				q.push(ofi);
			}
		}
	}

	groups.chartFaces.resize(static_cast<std::size_t>(nextChart));
	for (int fi = 0; fi < qCount; ++fi)
	{
		const int cid = faceChart[static_cast<std::size_t>(fi)];
		if (cid >= 0)
		{
			groups.chartFaces[static_cast<std::size_t>(cid)].push_back(fi);
		}
	}
	return groups;
}

} // namespace meshrecon
} // namespace geoalgo
