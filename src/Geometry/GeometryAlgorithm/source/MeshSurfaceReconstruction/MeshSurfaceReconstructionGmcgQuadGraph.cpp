#include "MeshSurfaceReconstructionGmcgQuadGraph.h"

#include <algorithm>
#include <cstdint>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

int64_t undirectedEdgeKey(const int a, const int b)
{
	const int lo = std::min(a, b);
	const int hi = std::max(a, b);
	return (static_cast<int64_t>(lo) << 32) | static_cast<uint32_t>(hi);
}

} // namespace

GmcgQuadGraph GmcgQuadGraph::build(const QuadMeshLite& mesh)
{
	GmcgQuadGraph graph;
	const int qCount = static_cast<int>(mesh.quadFaces.size() / 4U);
	if (qCount < 1 || mesh.vertices.empty())
	{
		return graph;
	}
	graph.quadCount = qCount;
	graph.halfEdges.resize(static_cast<std::size_t>(qCount) * 4U);
	graph.faceToHalfEdge.resize(static_cast<std::size_t>(qCount));
	const int vCount = static_cast<int>(mesh.vertices.size() / 3U);
	graph.vertexHalfEdges.resize(static_cast<std::size_t>(vCount));

	std::unordered_map<int64_t, int> edgeToHalfEdge;
	edgeToHalfEdge.reserve(static_cast<std::size_t>(qCount) * 4U);

	for (int fi = 0; fi < qCount; ++fi)
	{
		const std::size_t fb = static_cast<std::size_t>(fi) * 4U;
		const int baseHe = fi * 4;
		graph.faceToHalfEdge[static_cast<std::size_t>(fi)] = baseHe;
		for (int e = 0; e < 4; ++e)
		{
			const int he = baseHe + e;
			const int from = mesh.quadFaces[fb + static_cast<std::size_t>(e)];
			const int to = mesh.quadFaces[fb + static_cast<std::size_t>((e + 1) % 4)];
			GmcgHalfEdge& h = graph.halfEdges[static_cast<std::size_t>(he)];
			h.face = fi;
			h.edgeInFace = e;
			h.from = from;
			h.to = to;
			h.next = baseHe + ((e + 1) % 4);
			if (from >= 0 && from < vCount)
			{
				graph.vertexHalfEdges[static_cast<std::size_t>(from)].push_back(he);
			}
			const int64_t key = undirectedEdgeKey(from, to);
			auto it = edgeToHalfEdge.find(key);
			if (it == edgeToHalfEdge.end())
			{
				edgeToHalfEdge.emplace(key, he);
				h.twin = -1;
			}
			else
			{
				const int other = it->second;
				h.twin = other;
				graph.halfEdges[static_cast<std::size_t>(other)].twin = he;
			}
		}
	}
	return graph;
}

int GmcgQuadGraph::advanceMotorcycle(const int halfEdge) const
{
	if (halfEdge < 0 || static_cast<std::size_t>(halfEdge) >= halfEdges.size())
	{
		return -1;
	}
	const int twin = halfEdges[static_cast<std::size_t>(halfEdge)].twin;
	if (twin < 0)
	{
		return -1;
	}
	const int tNext = halfEdges[static_cast<std::size_t>(twin)].next;
	const int tNextNext = halfEdges[static_cast<std::size_t>(tNext)].next;
	return tNextNext;
}

bool GmcgQuadGraph::isBoundaryHalfEdge(const int halfEdge) const
{
	if (halfEdge < 0 || static_cast<std::size_t>(halfEdge) >= halfEdges.size())
	{
		return true;
	}
	return halfEdges[static_cast<std::size_t>(halfEdge)].twin < 0;
}

} // namespace meshrecon
} // namespace geoalgo
