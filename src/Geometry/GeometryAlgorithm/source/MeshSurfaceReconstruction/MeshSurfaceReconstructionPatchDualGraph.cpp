/// @file MeshSurfaceReconstructionPatchDualGraph.cpp
/// @brief MeshSurfaceReconstructionPatchDualGraph 实现

#include "MeshSurfaceReconstructionPatchDualGraph.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
namespace
{
struct GenEdge
{
	int id = -1;
	int patchA = -1;
	int patchB = -1;
	int cornerA = -1;
	int cornerB = -1;
	std::vector<int64_t> meshEdgeKeys;
	bool isFeature = false;
	bool isBoundary = false;
	double endpointDistance = 0.0;
};

struct DualCorner
{
	int meshVertex = -1;
	std::vector<int> genEdgeIds;
};

PartitionVec3d patchAverageNormal(const std::vector<int>& faces, const std::vector<PartitionVec3d>& faceNormals)
{
	PartitionVec3d sum{0, 0, 0};
	for (const int f : faces)
	{
		sum = sum + faceNormals[static_cast<std::size_t>(f)];
	}
	return sum.normalized();
}

void rebuildPatchFaceLists(const int faceCount, const std::vector<int>& faceToPatch,
						   std::vector<std::vector<int>>& patchFaces)
{
	patchFaces.clear();
	for (int f = 0; f < faceCount; ++f)
	{
		const int p = faceToPatch[static_cast<std::size_t>(f)];
		if (p < 0)
		{
			continue;
		}
		if (static_cast<std::size_t>(p) >= patchFaces.size())
		{
			patchFaces.resize(static_cast<std::size_t>(p + 1));
		}
		patchFaces[static_cast<std::size_t>(p)].push_back(f);
	}
	patchFaces.erase(std::remove_if(patchFaces.begin(), patchFaces.end(),
									[](const std::vector<int>& faces) { return faces.empty(); }),
					 patchFaces.end());
}

void compactFaceToPatch(std::vector<int>& faceToPatch)
{
	std::unordered_map<int, int> remap;
	int next = 0;
	for (int& p : faceToPatch)
	{
		if (p < 0)
		{
			continue;
		}
		auto it = remap.find(p);
		if (it == remap.end())
		{
			remap[p] = next++;
			p = next - 1;
		}
		else
		{
			p = it->second;
		}
	}
}

std::vector<std::unordered_set<int>> buildPatchNeighbors(const int faceCount, const std::vector<int>& faceToPatch,
														 const std::vector<std::vector<int>>& fullAdj)
{
	std::vector<std::unordered_set<int>> neighbors;
	for (int f = 0; f < faceCount; ++f)
	{
		const int p = faceToPatch[static_cast<std::size_t>(f)];
		if (p < 0)
		{
			continue;
		}
		if (static_cast<std::size_t>(p) >= neighbors.size())
		{
			neighbors.resize(static_cast<std::size_t>(p + 1));
		}
		for (const int nb : fullAdj[static_cast<std::size_t>(f)])
		{
			const int np = faceToPatch[static_cast<std::size_t>(nb)];
			if (np >= 0 && np != p)
			{
				neighbors[static_cast<std::size_t>(p)].insert(np);
			}
		}
	}
	return neighbors;
}

double sharedBoundaryLength(const int patchA, const int patchB, const std::vector<int>& faceToPatch,
							const IndexedMeshLite& mesh,
							const std::unordered_map<int64_t, std::pair<int, int>>& edgeToFaces)
{
	double len = 0.0;
	for (const auto& kv : edgeToFaces)
	{
		const int f0 = kv.second.first;
		const int f1 = kv.second.second;
		if (f0 < 0 || f1 < 0)
		{
			continue;
		}
		const int p0 = faceToPatch[static_cast<std::size_t>(f0)];
		const int p1 = faceToPatch[static_cast<std::size_t>(f1)];
		if ((p0 == patchA && p1 == patchB) || (p0 == patchB && p1 == patchA))
		{
			const int64_t key = kv.first;
			const int v0 = static_cast<int>(key >> 32);
			const int v1 = static_cast<int>(key & 0xffffffffLL);
			len += (readPartitionV(mesh.vertices, v0) - readPartitionV(mesh.vertices, v1)).length();
		}
	}
	return len;
}

void mergePatchInto(const int src, const int dst, std::vector<int>& faceToPatch)
{
	for (int& p : faceToPatch)
	{
		if (p == src)
		{
			p = dst;
		}
	}
}

bool step1MergeLowValencePatches(const IndexedMeshLite& mesh, const std::vector<std::vector<int>>& fullAdj,
								 const std::unordered_map<int64_t, std::pair<int, int>>& edgeToFaces,
								 std::vector<int>& faceToPatch)
{
	const int faceCount = static_cast<int>(faceToPatch.size());
	bool changed = false;
	for (int pass = 0; pass < faceCount; ++pass)
	{
		const auto neighbors = buildPatchNeighbors(faceCount, faceToPatch, fullAdj);
		if (neighbors.empty())
		{
			break;
		}
		int mergeSrc = -1;
		int mergeDst = -1;
		for (std::size_t pi = 0; pi < neighbors.size(); ++pi)
		{
			if (neighbors[pi].size() == 1U)
			{
				mergeSrc = static_cast<int>(pi);
				mergeDst = *neighbors[pi].begin();
				break;
			}
		}
		if (mergeSrc < 0)
		{
			for (std::size_t pi = 0; pi < neighbors.size(); ++pi)
			{
				if (neighbors[pi].size() != 2U)
				{
					continue;
				}
				const int n0 = *neighbors[pi].begin();
				const int n1 = *std::next(neighbors[pi].begin());
				const double l0 = sharedBoundaryLength(static_cast<int>(pi), n0, faceToPatch, mesh, edgeToFaces);
				const double l1 = sharedBoundaryLength(static_cast<int>(pi), n1, faceToPatch, mesh, edgeToFaces);
				mergeSrc = static_cast<int>(pi);
				mergeDst = (l0 >= l1) ? n0 : n1;
				break;
			}
		}
		if (mergeSrc < 0)
		{
			break;
		}
		mergePatchInto(mergeSrc, mergeDst, faceToPatch);
		compactFaceToPatch(faceToPatch);
		changed = true;
	}
	return changed;
}

bool isFeaturePair(const int patchA, const int patchB, const std::vector<std::vector<int>>& patchFaces,
				   const std::vector<PartitionVec3d>& faceNormals, const double featureAngleDeg)
{
	const PartitionVec3d na = patchAverageNormal(patchFaces[static_cast<std::size_t>(patchA)], faceNormals);
	const PartitionVec3d nb = patchAverageNormal(patchFaces[static_cast<std::size_t>(patchB)], faceNormals);
	const double dotVal = std::max(-1.0, std::min(1.0, na.dot(nb)));
	const double angleDeg = std::acos(dotVal) * (180.0 / 3.14159265358979323846);
	return angleDeg > featureAngleDeg;
}

void buildDualCornersAndGenEdges(const IndexedMeshLite& mesh, const int faceCount, const std::vector<int>& faceToPatch,
								 const std::vector<std::vector<int>>& patchFaces,
								 const std::vector<PartitionVec3d>& faceNormals, const MeshAdjacency& adj,
								 const double featureAngleDeg, std::vector<DualCorner>& corners,
								 std::vector<GenEdge>& genEdges, std::vector<std::vector<int>>& patchSideGenEdges)
{
	corners.clear();
	genEdges.clear();
	patchSideGenEdges.assign(patchFaces.size(), {});

	std::unordered_map<int, int> vertexToCorner;
	auto cornerIndex = [&](const int v) -> int
	{
		auto it = vertexToCorner.find(v);
		if (it != vertexToCorner.end())
		{
			return it->second;
		}
		const int idx = static_cast<int>(corners.size());
		vertexToCorner[v] = idx;
		DualCorner c;
		c.meshVertex = v;
		corners.push_back(c);
		return idx;
	};

	struct DirectedBoundary
	{
		int64_t key = 0;
		int fromV = -1;
		int toV = -1;
		int patchA = -1;
		int patchB = -1;
	};

	std::vector<DirectedBoundary> directed;
	directed.reserve(adj.edgeToFaces.size() * 2U);
	for (const auto& kv : adj.edgeToFaces)
	{
		const int f0 = kv.second.first;
		const int f1 = kv.second.second;
		const int64_t key = kv.first;
		const int v0 = static_cast<int>(key >> 32);
		const int v1 = static_cast<int>(key & 0xffffffffLL);
		if (f0 < 0)
		{
			continue;
		}
		const int p0 = faceToPatch[static_cast<std::size_t>(f0)];
		if (f1 < 0)
		{
			DirectedBoundary d;
			d.key = key;
			d.fromV = v0;
			d.toV = v1;
			d.patchA = p0;
			d.patchB = -1;
			directed.push_back(d);
			d.fromV = v1;
			d.toV = v0;
			directed.push_back(d);
			continue;
		}
		const int p1 = faceToPatch[static_cast<std::size_t>(f1)];
		if (p0 == p1)
		{
			continue;
		}
		DirectedBoundary ab;
		ab.key = key;
		ab.fromV = v0;
		ab.toV = v1;
		ab.patchA = p0;
		ab.patchB = p1;
		directed.push_back(ab);
		DirectedBoundary ba = ab;
		ba.fromV = v1;
		ba.toV = v0;
		ba.patchA = p1;
		ba.patchB = p0;
		directed.push_back(ba);
	}

	std::unordered_map<int, std::vector<int>> vertexOut;
	for (int i = 0; i < static_cast<int>(directed.size()); ++i)
	{
		vertexOut[directed[static_cast<std::size_t>(i)].fromV].push_back(i);
	}

	std::unordered_map<int, bool> isCornerVertex;
	for (const auto& kv : vertexOut)
	{
		std::unordered_set<int64_t> pairKeys;
		for (const int di : kv.second)
		{
			const DirectedBoundary& d = directed[static_cast<std::size_t>(di)];
			const int lo = std::min(d.patchA, d.patchB);
			const int hi = std::max(d.patchA, d.patchB);
			const int64_t pk = (static_cast<int64_t>(lo + 2) << 32) | static_cast<int64_t>(hi + 2);
			pairKeys.insert(pk);
		}
		isCornerVertex[kv.first] = pairKeys.size() >= 2U;
	}

	std::vector<char> usedDirected(directed.size(), 0);
	for (std::size_t pi = 0; pi < patchFaces.size(); ++pi)
	{
		int startDi = -1;
		for (int di = 0; di < static_cast<int>(directed.size()); ++di)
		{
			if (usedDirected[static_cast<std::size_t>(di)] != 0)
			{
				continue;
			}
			if (directed[static_cast<std::size_t>(di)].patchA == static_cast<int>(pi))
			{
				startDi = di;
				break;
			}
		}
		if (startDi < 0)
		{
			continue;
		}
		int di = startDi;
		int guard = 0;
		while (usedDirected[static_cast<std::size_t>(di)] == 0 && guard++ < static_cast<int>(directed.size()))
		{
			const int startCornerV = directed[static_cast<std::size_t>(di)].fromV;
			const int cornerA = cornerIndex(startCornerV);
			std::vector<int64_t> chainKeys;
			chainKeys.push_back(directed[static_cast<std::size_t>(di)].key);
			usedDirected[static_cast<std::size_t>(di)] = 1;
			int curV = directed[static_cast<std::size_t>(di)].toV;
			const int sidePatchA = static_cast<int>(pi);
			const int sidePatchB = directed[static_cast<std::size_t>(di)].patchB;

			while (!isCornerVertex[curV])
			{
				int nextDi = -1;
				for (const int cand : vertexOut[curV])
				{
					if (usedDirected[static_cast<std::size_t>(cand)] != 0)
					{
						continue;
					}
					const DirectedBoundary& d = directed[static_cast<std::size_t>(cand)];
					if (d.patchA == sidePatchA && d.patchB == sidePatchB)
					{
						nextDi = cand;
						break;
					}
				}
				if (nextDi < 0)
				{
					break;
				}
				chainKeys.push_back(directed[static_cast<std::size_t>(nextDi)].key);
				usedDirected[static_cast<std::size_t>(nextDi)] = 1;
				curV = directed[static_cast<std::size_t>(nextDi)].toV;
			}

			const int cornerB = cornerIndex(curV);
			GenEdge ge;
			ge.id = static_cast<int>(genEdges.size());
			ge.patchA = sidePatchA;
			ge.patchB = sidePatchB;
			ge.cornerA = cornerA;
			ge.cornerB = cornerB;
			ge.meshEdgeKeys = std::move(chainKeys);
			ge.isBoundary = sidePatchB < 0;
			if (!ge.isBoundary && sidePatchB < static_cast<int>(patchFaces.size()))
			{
				ge.isFeature = isFeaturePair(sidePatchA, sidePatchB, patchFaces, faceNormals, featureAngleDeg);
			}
			const PartitionVec3d pa =
				readPartitionV(mesh.vertices, corners[static_cast<std::size_t>(cornerA)].meshVertex);
			const PartitionVec3d pb =
				readPartitionV(mesh.vertices, corners[static_cast<std::size_t>(cornerB)].meshVertex);
			ge.endpointDistance = (pa - pb).length();
			genEdges.push_back(ge);
			corners[static_cast<std::size_t>(cornerA)].genEdgeIds.push_back(ge.id);
			corners[static_cast<std::size_t>(cornerB)].genEdgeIds.push_back(ge.id);
			patchSideGenEdges[pi].push_back(ge.id);

			di = -1;
			for (int ndi = 0; ndi < static_cast<int>(directed.size()); ++ndi)
			{
				if (usedDirected[static_cast<std::size_t>(ndi)] != 0)
				{
					continue;
				}
				if (directed[static_cast<std::size_t>(ndi)].patchA == static_cast<int>(pi))
				{
					di = ndi;
					break;
				}
			}
			if (di < 0 || di == startDi)
			{
				break;
			}
		}
	}
}

bool collapsePatchPair(const int keepPatch, const int dropPatch, std::vector<int>& faceToPatch,
					   std::vector<std::vector<int>>& patchFaces)
{
	if (keepPatch < 0 || dropPatch < 0 || keepPatch == dropPatch ||
		static_cast<std::size_t>(keepPatch) >= patchFaces.size() ||
		static_cast<std::size_t>(dropPatch) >= patchFaces.size())
	{
		return false;
	}
	if (patchFaces[static_cast<std::size_t>(dropPatch)].empty())
	{
		return false;
	}
	for (const int f : patchFaces[static_cast<std::size_t>(dropPatch)])
	{
		faceToPatch[static_cast<std::size_t>(f)] = keepPatch;
	}
	patchFaces[static_cast<std::size_t>(keepPatch)].insert(patchFaces[static_cast<std::size_t>(keepPatch)].end(),
														   patchFaces[static_cast<std::size_t>(dropPatch)].begin(),
														   patchFaces[static_cast<std::size_t>(dropPatch)].end());
	patchFaces[static_cast<std::size_t>(dropPatch)].clear();
	return true;
}

double triangleQuality(const PartitionVec3d& a, const PartitionVec3d& b, const PartitionVec3d& c)
{
	const double e0 = (b - a).length();
	const double e1 = (c - b).length();
	const double e2 = (a - c).length();
	const double sumSq = e0 * e0 + e1 * e1 + e2 * e2;
	if (sumSq < 1e-18)
	{
		return 0.0;
	}
	const double area2 = crossPartitionVec(b - a, c - a).length();
	return (4.0 * std::sqrt(3.0) * (area2 * 0.5)) / sumSq;
}

double quadQualityFromCorners(const IndexedMeshLite& mesh, const int v0, const int v1, const int v2, const int v3)
{
	const PartitionVec3d p0 = readPartitionV(mesh.vertices, v0);
	const PartitionVec3d p1 = readPartitionV(mesh.vertices, v1);
	const PartitionVec3d p2 = readPartitionV(mesh.vertices, v2);
	const PartitionVec3d p3 = readPartitionV(mesh.vertices, v3);
	return (triangleQuality(p0, p1, p2) + triangleQuality(p0, p2, p3)) * 0.5;
}

double bestQuadQualityFromMeshVerts(const IndexedMeshLite& mesh, const int v0, const int v1, const int v2, const int v3)
{
	const int verts[4] = {v0, v1, v2, v3};
	double best = -1.0;
	for (int shift = 0; shift < 4; ++shift)
	{
		const double q = quadQualityFromCorners(mesh, verts[shift], verts[(shift + 1) % 4], verts[(shift + 2) % 4],
												verts[(shift + 3) % 4]);
		if (q > best)
		{
			best = q;
		}
	}
	return best;
}

bool isGenEdgeBetweenPatches(const GenEdge& ge, const int patchA, const int patchB)
{
	return (ge.patchA == patchA && ge.patchB == patchB) || (ge.patchA == patchB && ge.patchB == patchA);
}

bool genEdgeOnPatch(const GenEdge& ge, const int patchId);

void collectNeighborValence3Corners(const int trianglePatch, const int neighborPatch, const int sharedCornerA,
									const int sharedCornerB, const std::vector<int>& neighborSideGenEdges,
									const std::vector<GenEdge>& genEdges, const std::vector<DualCorner>& corners,
									std::vector<int>& outCornerIds)
{
	outCornerIds.clear();
	for (const int geId : neighborSideGenEdges)
	{
		if (geId < 0 || static_cast<std::size_t>(geId) >= genEdges.size())
		{
			continue;
		}
		const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
		if (isGenEdgeBetweenPatches(ge, trianglePatch, neighborPatch))
		{
			continue;
		}
		if (!genEdgeOnPatch(ge, neighborPatch))
		{
			continue;
		}
		for (const int ci : {ge.cornerA, ge.cornerB})
		{
			if (ci == sharedCornerA || ci == sharedCornerB)
			{
				continue;
			}
			if (ci < 0 || static_cast<std::size_t>(ci) >= corners.size())
			{
				continue;
			}
			if (static_cast<int>(corners[static_cast<std::size_t>(ci)].genEdgeIds.size()) != 3)
			{
				continue;
			}
			outCornerIds.push_back(ci);
		}
	}
	std::sort(outCornerIds.begin(), outCornerIds.end());
	outCornerIds.erase(std::unique(outCornerIds.begin(), outCornerIds.end()), outCornerIds.end());
}

int neighborPatchAcross(const GenEdge& ge, const int patchId)
{
	if (ge.patchA == patchId)
	{
		return ge.patchB;
	}
	if (ge.patchB == patchId)
	{
		return ge.patchA;
	}
	return -1;
}

bool genEdgeOnPatch(const GenEdge& ge, const int patchId)
{
	return ge.patchA == patchId || ge.patchB == patchId;
}

void collectPatchCorners(const int patchId, const std::vector<int>& sideGenEdgeIds,
						 const std::vector<GenEdge>& genEdges, std::vector<int>& outCornerIds)
{
	outCornerIds.clear();
	for (const int geId : sideGenEdgeIds)
	{
		if (geId < 0 || static_cast<std::size_t>(geId) >= genEdges.size())
		{
			continue;
		}
		const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
		if (!genEdgeOnPatch(ge, patchId))
		{
			continue;
		}
		outCornerIds.push_back(ge.cornerA);
		outCornerIds.push_back(ge.cornerB);
	}
	std::sort(outCornerIds.begin(), outCornerIds.end());
	outCornerIds.erase(std::unique(outCornerIds.begin(), outCornerIds.end()), outCornerIds.end());
}

int countValence3Corners(const std::vector<int>& cornerIds, const std::vector<DualCorner>& corners)
{
	int count = 0;
	for (const int ci : cornerIds)
	{
		if (ci >= 0 && static_cast<std::size_t>(ci) < corners.size() &&
			static_cast<int>(corners[static_cast<std::size_t>(ci)].genEdgeIds.size()) == 3)
		{
			++count;
		}
	}
	return count;
}

bool endpointSatisfiesCollapseConditionB(const int cornerIdx, const int geId, const std::vector<DualCorner>& corners,
										 const std::vector<GenEdge>& genEdges, const double lengthRatio)
{
	if (cornerIdx < 0 || static_cast<std::size_t>(cornerIdx) >= corners.size())
	{
		return false;
	}
	if (geId < 0 || static_cast<std::size_t>(geId) >= genEdges.size())
	{
		return false;
	}
	const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
	const double minOtherLen = ge.endpointDistance * lengthRatio;
	int featureOrBoundary = 0;
	for (const int otherId : corners[static_cast<std::size_t>(cornerIdx)].genEdgeIds)
	{
		if (otherId == geId)
		{
			continue;
		}
		if (otherId < 0 || static_cast<std::size_t>(otherId) >= genEdges.size())
		{
			continue;
		}
		const GenEdge& other = genEdges[static_cast<std::size_t>(otherId)];
		if (other.isFeature || other.isBoundary)
		{
			++featureOrBoundary;
			continue;
		}
		if (other.endpointDistance <= minOtherLen + 1e-12)
		{
			return false;
		}
	}
	return featureOrBoundary <= 1;
}

bool canCollapseGenEdge(const GenEdge& ge, const std::vector<DualCorner>& corners, const std::vector<GenEdge>& genEdges,
						const MeshSurfaceReconstructParams& params)
{
	if (ge.isFeature || ge.isBoundary || ge.patchB < 0)
	{
		return false;
	}
	const int va = ge.cornerA;
	const int vb = ge.cornerB;
	if (va < 0 || vb < 0 || static_cast<std::size_t>(va) >= corners.size() ||
		static_cast<std::size_t>(vb) >= corners.size())
	{
		return false;
	}
	const int valenceSum = static_cast<int>(corners[static_cast<std::size_t>(va)].genEdgeIds.size()) +
						   static_cast<int>(corners[static_cast<std::size_t>(vb)].genEdgeIds.size());
	if (valenceSum > params.hybridCollapseValenceSumMax)
	{
		return false;
	}
	const double ratio = std::max(0.1, params.hybridCollapseLengthRatio);
	const bool okA = endpointSatisfiesCollapseConditionB(va, ge.id, corners, genEdges, ratio);
	const bool okB = endpointSatisfiesCollapseConditionB(vb, ge.id, corners, genEdges, ratio);
	return okA || okB;
}

double collapseQualityToCorner(const IndexedMeshLite& mesh, const int cornerIdx, const GenEdge& ge,
							   const std::vector<DualCorner>& corners, const std::vector<GenEdge>& genEdges)
{
	if (cornerIdx < 0 || static_cast<std::size_t>(cornerIdx) >= corners.size())
	{
		return -1.0;
	}
	const int meshP = corners[static_cast<std::size_t>(cornerIdx)].meshVertex;
	const PartitionVec3d p = readPartitionV(mesh.vertices, meshP);
	const int otherCorner = (ge.cornerA == cornerIdx) ? ge.cornerB : ge.cornerA;
	if (otherCorner < 0 || static_cast<std::size_t>(otherCorner) >= corners.size())
	{
		return -1.0;
	}
	const PartitionVec3d pOther =
		readPartitionV(mesh.vertices, corners[static_cast<std::size_t>(otherCorner)].meshVertex);
	double score = 0.0;
	int count = 0;
	for (const int geId : corners[static_cast<std::size_t>(cornerIdx)].genEdgeIds)
	{
		if (geId == ge.id || geId < 0 || static_cast<std::size_t>(geId) >= genEdges.size())
		{
			continue;
		}
		const GenEdge& og = genEdges[static_cast<std::size_t>(geId)];
		const int oc = (og.cornerA == cornerIdx) ? og.cornerB : og.cornerA;
		if (oc < 0 || static_cast<std::size_t>(oc) >= corners.size())
		{
			continue;
		}
		const PartitionVec3d pThird = readPartitionV(mesh.vertices, corners[static_cast<std::size_t>(oc)].meshVertex);
		score += triangleQuality(p, pOther, pThird);
		++count;
	}
	return count > 0 ? score / static_cast<double>(count) : 0.0;
}

bool collapseAcrossGenEdge(const IndexedMeshLite& mesh, const GenEdge& ge, const std::vector<DualCorner>& corners,
						   const std::vector<GenEdge>& genEdges, const MeshSurfaceReconstructParams& params,
						   std::vector<int>& faceToPatch, std::vector<std::vector<int>>& patchFaces)
{
	if (!canCollapseGenEdge(ge, corners, genEdges, params))
	{
		return false;
	}
	const double ratio = std::max(0.1, params.hybridCollapseLengthRatio);
	const bool okA = endpointSatisfiesCollapseConditionB(ge.cornerA, ge.id, corners, genEdges, ratio);
	const bool okB = endpointSatisfiesCollapseConditionB(ge.cornerB, ge.id, corners, genEdges, ratio);
	int keepPatch = ge.patchA;
	int dropPatch = ge.patchB;
	if (okA && okB)
	{
		const double qa = collapseQualityToCorner(mesh, ge.cornerA, ge, corners, genEdges);
		const double qb = collapseQualityToCorner(mesh, ge.cornerB, ge, corners, genEdges);
		if (qb > qa)
		{
			keepPatch = ge.patchB;
			dropPatch = ge.patchA;
		}
	}
	else if (okB && !okA)
	{
		keepPatch = ge.patchB;
		dropPatch = ge.patchA;
	}
	return collapsePatchPair(keepPatch, dropPatch, faceToPatch, patchFaces);
}

const GenEdge* findGenEdgeBetweenCorners(const int cornerA, const int cornerB, const std::vector<int>& sideGenEdgeIds,
										 const std::vector<GenEdge>& genEdges)
{
	for (const int geId : sideGenEdgeIds)
	{
		if (geId < 0 || static_cast<std::size_t>(geId) >= genEdges.size())
		{
			continue;
		}
		const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
		if ((ge.cornerA == cornerA && ge.cornerB == cornerB) || (ge.cornerA == cornerB && ge.cornerB == cornerA))
		{
			return &ge;
		}
	}
	return nullptr;
}

std::vector<const GenEdge*> genEdgesAtCornerOnPatch(const int cornerIdx, const int patchId,
													const std::vector<int>& sideGenEdgeIds,
													const std::vector<GenEdge>& genEdges)
{
	std::vector<const GenEdge*> out;
	for (const int geId : sideGenEdgeIds)
	{
		if (geId < 0 || static_cast<std::size_t>(geId) >= genEdges.size())
		{
			continue;
		}
		const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
		if (!genEdgeOnPatch(ge, patchId))
		{
			continue;
		}
		if (ge.cornerA == cornerIdx || ge.cornerB == cornerIdx)
		{
			out.push_back(&ge);
		}
	}
	return out;
}

bool step332DeleteSpecialGenEdges(const IndexedMeshLite& mesh, const std::vector<DualCorner>& corners,
								  const std::vector<GenEdge>& genEdges,
								  const std::vector<std::vector<int>>& patchSideGenEdges,
								  const MeshSurfaceReconstructParams& params, std::vector<int>& faceToPatch,
								  std::vector<std::vector<int>>& patchFaces)
{
	for (const GenEdge& ge : genEdges)
	{
		if (ge.isFeature || ge.isBoundary || ge.patchB < 0)
		{
			continue;
		}
		const int ca = ge.cornerA;
		const int cb = ge.cornerB;
		if (ca < 0 || cb < 0)
		{
			continue;
		}
		if (static_cast<int>(corners[static_cast<std::size_t>(ca)].genEdgeIds.size()) != 3 ||
			static_cast<int>(corners[static_cast<std::size_t>(cb)].genEdgeIds.size()) != 3)
		{
			continue;
		}
		const int sidesA = static_cast<int>(patchSideGenEdges[static_cast<std::size_t>(ge.patchA)].size());
		const int sidesB = static_cast<int>(patchSideGenEdges[static_cast<std::size_t>(ge.patchB)].size());
		if (!((sidesA == 4 && sidesB == 4) || (sidesA == 4 && sidesB == 5) || (sidesA == 5 && sidesB == 4)))
		{
			continue;
		}
		if (collapseAcrossGenEdge(mesh, ge, corners, genEdges, params, faceToPatch, patchFaces))
		{
			return true;
		}
	}
	return false;
}

bool step333SortedCollapse(const IndexedMeshLite& mesh, const std::vector<DualCorner>& corners,
						   const std::vector<GenEdge>& genEdges, const MeshSurfaceReconstructParams& params,
						   std::vector<int>& faceToPatch, std::vector<std::vector<int>>& patchFaces)
{
	std::vector<int> order(genEdges.size());
	for (std::size_t i = 0; i < genEdges.size(); ++i)
	{
		order[i] = static_cast<int>(i);
	}
	std::sort(order.begin(), order.end(),
			  [&](const int a, const int b)
			  {
				  return genEdges[static_cast<std::size_t>(a)].endpointDistance <
						 genEdges[static_cast<std::size_t>(b)].endpointDistance;
			  });
	for (const int geId : order)
	{
		const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
		if (collapseAcrossGenEdge(mesh, ge, corners, genEdges, params, faceToPatch, patchFaces))
		{
			return true;
		}
	}
	return false;
}

bool tryCollapseLongestNonFeatureAtCorner(const IndexedMeshLite& mesh, const int patchId, const int cornerIdx,
										  const std::vector<int>& sideGenEdgeIds,
										  const std::vector<DualCorner>& corners, const std::vector<GenEdge>& genEdges,
										  const MeshSurfaceReconstructParams& params, std::vector<int>& faceToPatch,
										  std::vector<std::vector<int>>& patchFaces)
{
	const std::vector<const GenEdge*> atCorner = genEdgesAtCornerOnPatch(cornerIdx, patchId, sideGenEdgeIds, genEdges);
	const GenEdge* best = nullptr;
	for (const GenEdge* ge : atCorner)
	{
		if (ge->isFeature || ge->isBoundary || ge->patchB < 0)
		{
			continue;
		}
		if (best == nullptr || ge->endpointDistance > best->endpointDistance)
		{
			best = ge;
		}
	}
	if (best == nullptr)
	{
		return false;
	}
	return collapseAcrossGenEdge(mesh, *best, corners, genEdges, params, faceToPatch, patchFaces);
}

bool step334TriPatchAdjust(const IndexedMeshLite& mesh, const std::vector<DualCorner>& corners,
						   const std::vector<GenEdge>& genEdges, const std::vector<std::vector<int>>& patchSideGenEdges,
						   const MeshSurfaceReconstructParams& params, std::vector<int>& faceToPatch,
						   std::vector<std::vector<int>>& patchFaces)
{
	for (std::size_t pi = 0; pi < patchSideGenEdges.size(); ++pi)
	{
		if (patchSideGenEdges[pi].size() != 3U)
		{
			continue;
		}
		const int patchId = static_cast<int>(pi);
		std::vector<int> patchCorners;
		collectPatchCorners(patchId, patchSideGenEdges[pi], genEdges, patchCorners);
		if (patchCorners.size() != 3U)
		{
			continue;
		}
		const int v3Count = countValence3Corners(patchCorners, corners);

		// §3.3-4a：单个度 3 角点，删较长非特征边
		if (v3Count == 1)
		{
			for (const int ci : patchCorners)
			{
				if (static_cast<int>(corners[static_cast<std::size_t>(ci)].genEdgeIds.size()) != 3)
				{
					continue;
				}
				if (tryCollapseLongestNonFeatureAtCorner(mesh, patchId, ci, patchSideGenEdges[pi], corners, genEdges,
														 params, faceToPatch, patchFaces))
				{
					return true;
				}
			}
		}

		// §3.3-4b：两个度 3 角点
		if (v3Count == 2)
		{
			int c3a = -1;
			int c3b = -1;
			for (const int ci : patchCorners)
			{
				if (static_cast<int>(corners[static_cast<std::size_t>(ci)].genEdgeIds.size()) == 3)
				{
					if (c3a < 0)
					{
						c3a = ci;
					}
					else
					{
						c3b = ci;
					}
				}
			}
			if (c3a >= 0 && c3b >= 0)
			{
				const GenEdge* connecting = findGenEdgeBetweenCorners(c3a, c3b, patchSideGenEdges[pi], genEdges);
				if (connecting != nullptr && !connecting->isFeature && !connecting->isBoundary &&
					collapseAcrossGenEdge(mesh, *connecting, corners, genEdges, params, faceToPatch, patchFaces))
				{
					return true;
				}
				const GenEdge* longestOther = nullptr;
				for (const int geId : patchSideGenEdges[pi])
				{
					const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
					if (connecting != nullptr && ge.id == connecting->id)
					{
						continue;
					}
					if (ge.isFeature || ge.isBoundary || ge.patchB < 0)
					{
						continue;
					}
					if (longestOther == nullptr || ge.endpointDistance > longestOther->endpointDistance)
					{
						longestOther = &ge;
					}
				}
				if (longestOther != nullptr &&
					collapseAcrossGenEdge(mesh, *longestOther, corners, genEdges, params, faceToPatch, patchFaces))
				{
					return true;
				}
			}
		}

		// §3.3-4c：三个度 3 角点，优先收缩边界广义边
		if (v3Count == 3)
		{
			const GenEdge* boundaryEdge = nullptr;
			const GenEdge* shortestInterior = nullptr;
			for (const int geId : patchSideGenEdges[pi])
			{
				const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
				if (ge.isFeature)
				{
					continue;
				}
				if (ge.isBoundary)
				{
					if (boundaryEdge == nullptr || ge.endpointDistance < boundaryEdge->endpointDistance)
					{
						boundaryEdge = &ge;
					}
				}
				else if (shortestInterior == nullptr || ge.endpointDistance < shortestInterior->endpointDistance)
				{
					shortestInterior = &ge;
				}
			}
			const GenEdge* target = boundaryEdge != nullptr ? boundaryEdge : shortestInterior;
			if (target != nullptr &&
				collapseAcrossGenEdge(mesh, *target, corners, genEdges, params, faceToPatch, patchFaces))
			{
				return true;
			}
		}

		// §3.3-4d：无度 3 角点，邻域度 3 角点 + 四边形质量择优
		if (v3Count == 0 && patchCorners.size() == 3U)
		{
			double bestScore = -1.0;
			const GenEdge* bestGe = nullptr;
			for (const int geId : patchSideGenEdges[pi])
			{
				const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
				if (ge.isFeature || ge.isBoundary || ge.patchB < 0)
				{
					continue;
				}
				const int nb = neighborPatchAcross(ge, patchId);
				if (nb < 0 || static_cast<std::size_t>(nb) >= patchSideGenEdges.size())
				{
					continue;
				}
				if (patchSideGenEdges[static_cast<std::size_t>(nb)].size() <= 4U)
				{
					continue;
				}
				const int ca = ge.cornerA;
				const int cb = ge.cornerB;
				if (ca < 0 || cb < 0)
				{
					continue;
				}
				int thirdCorner = -1;
				for (const int ci : patchCorners)
				{
					if (ci != ca && ci != cb)
					{
						thirdCorner = ci;
						break;
					}
				}
				if (thirdCorner < 0)
				{
					continue;
				}
				std::vector<int> neighborV3Corners;
				collectNeighborValence3Corners(patchId, nb, ca, cb, patchSideGenEdges[static_cast<std::size_t>(nb)],
											   genEdges, corners, neighborV3Corners);
				if (neighborV3Corners.empty())
				{
					continue;
				}
				const int va = corners[static_cast<std::size_t>(ca)].meshVertex;
				const int vb = corners[static_cast<std::size_t>(cb)].meshVertex;
				const int vc = corners[static_cast<std::size_t>(thirdCorner)].meshVertex;
				double score = -1.0;
				for (const int nc : neighborV3Corners)
				{
					const int vd = corners[static_cast<std::size_t>(nc)].meshVertex;
					score = std::max(score, bestQuadQualityFromMeshVerts(mesh, va, vb, vc, vd));
				}
				if (score > bestScore)
				{
					bestScore = score;
					bestGe = &ge;
				}
			}
			if (bestGe != nullptr &&
				collapseAcrossGenEdge(mesh, *bestGe, corners, genEdges, params, faceToPatch, patchFaces))
			{
				return true;
			}
		}
	}

	// §3.3-4e：相邻两片均为三边时合并
	for (std::size_t pi = 0; pi < patchSideGenEdges.size(); ++pi)
	{
		if (patchSideGenEdges[pi].size() != 3U)
		{
			continue;
		}
		for (const int geId : patchSideGenEdges[pi])
		{
			const GenEdge& ge = genEdges[static_cast<std::size_t>(geId)];
			if (ge.isFeature || ge.isBoundary || ge.patchB < 0)
			{
				continue;
			}
			const int nb = neighborPatchAcross(ge, static_cast<int>(pi));
			if (nb < 0 || static_cast<std::size_t>(nb) >= patchSideGenEdges.size())
			{
				continue;
			}
			if (patchSideGenEdges[static_cast<std::size_t>(nb)].size() != 3U)
			{
				continue;
			}
			if (collapseAcrossGenEdge(mesh, ge, corners, genEdges, params, faceToPatch, patchFaces))
			{
				return true;
			}
		}
	}
	return false;
}

void countPatchSideStats(const std::vector<std::vector<int>>& patchSideGenEdges, HybridAdjustStats& stats)
{
	stats.triPatchCount = 0;
	stats.quadPatchCount = 0;
	stats.pentPatchCount = 0;
	stats.hexPatchCount = 0;
	for (const auto& sides : patchSideGenEdges)
	{
		if (sides.empty())
		{
			continue;
		}
		switch (static_cast<int>(sides.size()))
		{
		case 3:
			++stats.triPatchCount;
			break;
		case 4:
			++stats.quadPatchCount;
			break;
		case 5:
			++stats.pentPatchCount;
			break;
		case 6:
			++stats.hexPatchCount;
			break;
		default:
			break;
		}
	}
}

} // namespace

bool hybridApplyRegionAdjust(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
							 const std::vector<std::vector<int>>& fullAdj,
							 const std::vector<PartitionVec3d>& faceNormals, std::vector<int>& faceToPatch,
							 HybridAdjustStats& stats, std::string* errMsg)
{
	const int faceCount = static_cast<int>(faceToPatch.size());
	MeshAdjacency adj;
	adj.fullAdj = fullAdj;
	adj.edgeToFaces = buildMeshAdjacency(mesh, faceCount).edgeToFaces;

	step1MergeLowValencePatches(mesh, fullAdj, adj.edgeToFaces, faceToPatch);
	compactFaceToPatch(faceToPatch);

	const int maxPasses = std::max(1, params.hybridRegionAdjustMaxPasses);
	for (int pass = 0; pass < maxPasses; ++pass)
	{
		std::vector<std::vector<int>> patchFaces;
		rebuildPatchFaceLists(faceCount, faceToPatch, patchFaces);
		if (patchFaces.size() <= 1U)
		{
			break;
		}

		std::vector<DualCorner> corners;
		std::vector<GenEdge> genEdges;
		std::vector<std::vector<int>> patchSideGenEdges;
		buildDualCornersAndGenEdges(mesh, faceCount, faceToPatch, patchFaces, faceNormals, adj,
									params.hybridFeatureAngleDeg, corners, genEdges, patchSideGenEdges);

		bool changed = false;
		if (step332DeleteSpecialGenEdges(mesh, corners, genEdges, patchSideGenEdges, params, faceToPatch, patchFaces))
		{
			changed = true;
		}
		else if (step333SortedCollapse(mesh, corners, genEdges, params, faceToPatch, patchFaces))
		{
			changed = true;
		}
		else if (step334TriPatchAdjust(mesh, corners, genEdges, patchSideGenEdges, params, faceToPatch, patchFaces))
		{
			changed = true;
		}
		if (!changed)
		{
			countPatchSideStats(patchSideGenEdges, stats);
			break;
		}
		compactFaceToPatch(faceToPatch);
	}

	std::vector<std::vector<int>> finalPatchFaces;
	rebuildPatchFaceLists(faceCount, faceToPatch, finalPatchFaces);
	std::vector<DualCorner> corners;
	std::vector<GenEdge> genEdges;
	std::vector<std::vector<int>> patchSideGenEdges;
	buildDualCornersAndGenEdges(mesh, faceCount, faceToPatch, finalPatchFaces, faceNormals, adj,
								params.hybridFeatureAngleDeg, corners, genEdges, patchSideGenEdges);
	countPatchSideStats(patchSideGenEdges, stats);

	if (finalPatchFaces.empty())
	{
		if (errMsg)
		{
			*errMsg = "hybrid region adjust produced no patches";
		}
		return false;
	}
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
