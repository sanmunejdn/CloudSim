#include "MeshSurfaceReconstructionGmcgMotorcycleTrace.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

void markBarrier(std::vector<uint8_t>& barriers, const int he)
{
	if (he >= 0 && static_cast<std::size_t>(he) < barriers.size())
	{
		barriers[static_cast<std::size_t>(he)] = 1U;
	}
}

} // namespace

GmcgMotorcycleResult traceMotorcycles(const GmcgQuadGraph& graph)
{
	GmcgMotorcycleResult result;
	const int heCount = static_cast<int>(graph.halfEdges.size());
	result.barrierHalfEdges.assign(static_cast<std::size_t>(heCount), 0U);
	result.vertexValence.resize(graph.vertexHalfEdges.size(), 0);
	for (std::size_t vi = 0; vi < graph.vertexHalfEdges.size(); ++vi)
	{
		result.vertexValence[vi] = static_cast<int>(graph.vertexHalfEdges[vi].size());
	}

	struct ActiveMotor
	{
		int halfEdge = -1;
	};
	std::vector<ActiveMotor> active;
	active.reserve(256U);

	for (std::size_t vi = 0; vi < graph.vertexHalfEdges.size(); ++vi)
	{
		if (result.vertexValence[vi] == 4)
		{
			continue;
		}
		for (const int he : graph.vertexHalfEdges[vi])
		{
			active.push_back({he});
		}
	}

	const int maxSteps = heCount * 4 + 16;
	for (int step = 0; step < maxSteps && !active.empty(); ++step)
	{
		std::unordered_map<int, int> targetCount;
		targetCount.reserve(active.size() * 2U);
		std::vector<int> nextHalfEdges(active.size(), -1);
		for (std::size_t i = 0; i < active.size(); ++i)
		{
			const int nextHe = graph.advanceMotorcycle(active[i].halfEdge);
			nextHalfEdges[i] = nextHe;
			if (nextHe >= 0)
			{
				++targetCount[nextHe];
			}
		}

		std::vector<ActiveMotor> nextActive;
		nextActive.reserve(active.size());
		for (std::size_t i = 0; i < active.size(); ++i)
		{
			const int cur = active[i].halfEdge;
			const int nextHe = nextHalfEdges[i];
			markBarrier(result.barrierHalfEdges, cur);
			if (nextHe < 0)
			{
				continue;
			}
			if (result.barrierHalfEdges[static_cast<std::size_t>(nextHe)] != 0U)
			{
				markBarrier(result.barrierHalfEdges, nextHe);
				continue;
			}
			if (targetCount[nextHe] > 1)
			{
				markBarrier(result.barrierHalfEdges, nextHe);
				continue;
			}
			nextActive.push_back({nextHe});
		}
		active.swap(nextActive);
	}

	// 边界天然为 barrier
	for (int he = 0; he < heCount; ++he)
	{
		if (graph.isBoundaryHalfEdge(he))
		{
			markBarrier(result.barrierHalfEdges, he);
		}
	}
	return result;
}

} // namespace meshrecon
} // namespace geoalgo
