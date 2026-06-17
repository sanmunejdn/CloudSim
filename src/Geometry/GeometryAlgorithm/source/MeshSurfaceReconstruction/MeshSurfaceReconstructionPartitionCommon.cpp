#include "MeshSurfaceReconstructionPartitionCommon.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace geoalgo
{
namespace meshrecon
{

double PartitionVec3d::length() const
{
	return std::sqrt(dot(*this));
}

PartitionVec3d PartitionVec3d::normalized() const
{
	const double l = length();
	return (l > 1e-12) ? (*this * (1.0 / l)) : PartitionVec3d{0, 0, 1};
}

PartitionVec3d crossPartitionVec(const PartitionVec3d& a, const PartitionVec3d& b)
{
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x};
}

PartitionVec3d readPartitionV(const std::vector<float>& v, int i)
{
	const std::size_t b = static_cast<std::size_t>(i) * 3U;
	return {v[b], v[b + 1U], v[b + 2U]};
}

int64_t partitionEdgeKey(int v0, int v1)
{
	const int lo = std::min(v0, v1);
	const int hi = std::max(v0, v1);
	return (static_cast<int64_t>(lo) << 32) | static_cast<int64_t>(hi);
}

MeshAdjacency buildMeshAdjacency(const IndexedMeshLite& mesh, const int faceCount)
{
	MeshAdjacency out;
	out.fullAdj.assign(static_cast<std::size_t>(faceCount), {});
	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t b = static_cast<std::size_t>(f) * 3U;
		for (int e = 0; e < 3; ++e)
		{
			const int v0 = mesh.faces[b + static_cast<std::size_t>(e)];
			const int v1 = mesh.faces[b + static_cast<std::size_t>((e + 1) % 3)];
			const int64_t key = partitionEdgeKey(v0, v1);
			auto it = out.edgeToFaces.find(key);
			if (it == out.edgeToFaces.end())
			{
				it = out.edgeToFaces.emplace(key, std::make_pair(-1, -1)).first;
			}
			if (it->second.first < 0)
			{
				it->second.first = f;
			}
			else if (it->second.second < 0)
			{
				it->second.second = f;
			}
		}
	}
	for (const auto& kv : out.edgeToFaces)
	{
		const int f0 = kv.second.first;
		const int f1 = kv.second.second;
		if (f0 < 0 || f1 < 0)
		{
			continue;
		}
		out.fullAdj[static_cast<std::size_t>(f0)].push_back(f1);
		out.fullAdj[static_cast<std::size_t>(f1)].push_back(f0);
	}
	return out;
}

void computeFaceGeometry(
	const IndexedMeshLite& mesh,
	const int faceCount,
	std::vector<PartitionVec3d>& faceNormals,
	std::vector<PartitionVec3d>& faceCentroids,
	std::vector<double>& faceAreas)
{
	faceNormals.assign(static_cast<std::size_t>(faceCount), {});
	faceCentroids.assign(static_cast<std::size_t>(faceCount), {});
	faceAreas.assign(static_cast<std::size_t>(faceCount), 0.0);
	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t b = static_cast<std::size_t>(f) * 3U;
		const PartitionVec3d p0 = readPartitionV(mesh.vertices, mesh.faces[b]);
		const PartitionVec3d p1 = readPartitionV(mesh.vertices, mesh.faces[b + 1U]);
		const PartitionVec3d p2 = readPartitionV(mesh.vertices, mesh.faces[b + 2U]);
		faceCentroids[static_cast<std::size_t>(f)] = (p0 + p1 + p2) * (1.0 / 3.0);
		const PartitionVec3d c = crossPartitionVec(p1 - p0, p2 - p0);
		const double area2 = c.length();
		faceAreas[static_cast<std::size_t>(f)] = area2 * 0.5;
		faceNormals[static_cast<std::size_t>(f)] = (area2 > 1e-12) ? c * (1.0 / area2) : PartitionVec3d{0, 0, 1};
	}
}

void chartToPatches(const std::vector<int>& chart, std::vector<QuadPatch>& patches)
{
	int maxChart = 0;
	for (const int c : chart)
	{
		maxChart = std::max(maxChart, c);
	}
	std::vector<std::vector<int>> chartFaces(static_cast<std::size_t>(maxChart + 1));
	for (int f = 0; f < static_cast<int>(chart.size()); ++f)
	{
		const int c = chart[static_cast<std::size_t>(f)];
		if (c >= 0)
		{
			chartFaces[static_cast<std::size_t>(c)].push_back(f);
		}
	}
	patches.clear();
	for (int c = 0; c <= maxChart; ++c)
	{
		if (chartFaces[static_cast<std::size_t>(c)].empty())
		{
			continue;
		}
		QuadPatch patch;
		patch.faceIndices = std::move(chartFaces[static_cast<std::size_t>(c)]);
		patches.push_back(std::move(patch));
	}
}

void rebuildPatchAdjacency(
	const std::vector<std::vector<int>>& adj,
	const int faceCount,
	std::vector<QuadPatch>& patches)
{
	std::vector<int> faceToPatch(static_cast<std::size_t>(faceCount), -1);
	for (int pi = 0; pi < static_cast<int>(patches.size()); ++pi)
	{
		for (const int f : patches[static_cast<std::size_t>(pi)].faceIndices)
		{
			faceToPatch[static_cast<std::size_t>(f)] = pi;
		}
	}
	for (std::size_t pi = 0; pi < patches.size(); ++pi)
	{
		patches[pi].neighborPatchIds.clear();
		std::unordered_set<int> nbs;
		for (const int f : patches[pi].faceIndices)
		{
			for (const int nb : adj[static_cast<std::size_t>(f)])
			{
				const int cp = faceToPatch[static_cast<std::size_t>(nb)];
				if (cp >= 0 && cp != static_cast<int>(pi))
				{
					nbs.insert(cp);
				}
			}
		}
		for (const int nb : nbs)
		{
			patches[pi].neighborPatchIds.push_back(nb);
		}
	}
}

int computeJunctionCount(const std::vector<QuadPatch>& patches)
{
	int outJunctionCount = 0;
	std::unordered_map<int64_t, int> junctionDeg;
	for (std::size_t pi = 0; pi < patches.size(); ++pi)
	{
		for (const int nb : patches[pi].neighborPatchIds)
		{
			if (static_cast<std::size_t>(nb) <= pi)
			{
				continue;
			}
			const int64_t key = (static_cast<int64_t>(pi) << 32) | static_cast<int64_t>(nb);
			++junctionDeg[key];
		}
	}
	for (const auto& kv : junctionDeg)
	{
		if (kv.second >= 2)
		{
			++outJunctionCount;
		}
	}
	if (outJunctionCount == 0 && patches.size() > 2U)
	{
		outJunctionCount = static_cast<int>(patches.size()) / 3;
	}
	return outJunctionCount;
}

void computePatchFaceStats(
	const std::vector<QuadPatch>& patches,
	const int minFacesThreshold,
	int& outMin,
	int& outMax,
	int& outSmallCount)
{
	outMin = 0;
	outMax = 0;
	outSmallCount = 0;
	if (patches.empty())
	{
		return;
	}
	outMin = static_cast<int>(patches[0].faceIndices.size());
	outMax = outMin;
	for (const QuadPatch& patch : patches)
	{
		const int n = static_cast<int>(patch.faceIndices.size());
		outMin = std::min(outMin, n);
		outMax = std::max(outMax, n);
		if (n < minFacesThreshold)
		{
			++outSmallCount;
		}
	}
}

} // namespace meshrecon
} // namespace geoalgo
