#include "MeshSurfaceReconstructionInternal.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

struct Vec3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	Vec3d operator+(const Vec3d& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3d operator-(const Vec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3d operator*(double s) const { return {x * s, y * s, z * s}; }
	double dot(const Vec3d& o) const { return x * o.x + y * o.y + z * o.z; }
	double length() const { return std::sqrt(dot(*this)); }
	Vec3d normalized() const
	{
		const double l = length();
		return (l > 1e-12) ? (*this * (1.0 / l)) : Vec3d{0, 0, 1};
	}
};

Vec3d crossv(const Vec3d& a, const Vec3d& b)
{
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x};
}

Vec3d readV(const std::vector<float>& v, int i)
{
	const std::size_t b = static_cast<std::size_t>(i) * 3U;
	return {v[b], v[b + 1U], v[b + 2U]};
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

void mergeTinyPatches(std::vector<QuadPatch>& patches, const int faceCount)
{
	const int minFaces = std::max(100, faceCount / 100);
	if (patches.size() <= 1U)
	{
		return;
	}
	std::size_t largest = 0U;
	for (std::size_t i = 1U; i < patches.size(); ++i)
	{
		if (patches[i].faceIndices.size() > patches[largest].faceIndices.size())
		{
			largest = i;
		}
	}
	bool merged = false;
	for (std::size_t i = 0U; i < patches.size(); ++i)
	{
		if (i == largest || patches[i].faceIndices.size() >= static_cast<std::size_t>(minFaces))
		{
			continue;
		}
		auto& dst = patches[largest].faceIndices;
		dst.insert(dst.end(), patches[i].faceIndices.begin(), patches[i].faceIndices.end());
		patches[i].faceIndices.clear();
		merged = true;
	}
	if (!merged)
	{
		return;
	}
	patches.erase(
		std::remove_if(
			patches.begin(),
			patches.end(),
			[](const QuadPatch& p) { return p.faceIndices.empty(); }),
		patches.end());
}

} // namespace

bool soupToIndexed(const std::vector<float>& soup, IndexedMeshLite& out, std::string* errMsg)
{
	if (soup.empty() || soup.size() % 9U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid triangle soup";
		}
		return false;
	}
	struct Key
	{
		int64_t x, y, z;
		bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
	};
	struct Hash
	{
		std::size_t operator()(const Key& k) const
		{
			return static_cast<std::size_t>(k.x ^ (k.y << 16) ^ (k.z << 32));
		}
	};
	std::unordered_map<Key, int, Hash> map;
	constexpr double scale = 1000.0;
	out.vertices.clear();
	out.faces.clear();
	const std::size_t triCount = soup.size() / 9U;
	for (std::size_t t = 0; t < triCount; ++t)
	{
		int idx[3];
		for (int c = 0; c < 3; ++c)
		{
			const std::size_t b = t * 9U + static_cast<std::size_t>(c) * 3U;
			Key key{
				static_cast<int64_t>(std::round(soup[b] * scale)),
				static_cast<int64_t>(std::round(soup[b + 1U] * scale)),
				static_cast<int64_t>(std::round(soup[b + 2U] * scale))};
			auto it = map.find(key);
			if (it == map.end())
			{
				const int ni = static_cast<int>(out.vertices.size() / 3U);
				map[key] = ni;
				out.vertices.push_back(soup[b]);
				out.vertices.push_back(soup[b + 1U]);
				out.vertices.push_back(soup[b + 2U]);
				idx[c] = ni;
			}
			else
			{
				idx[c] = it->second;
			}
		}
		out.faces.push_back(idx[0]);
		out.faces.push_back(idx[1]);
		out.faces.push_back(idx[2]);
	}
	return true;
}

bool partitionQuadDomains(
	const IndexedMeshLite& mesh,
	const MeshSurfaceReconstructParams& params,
	std::vector<QuadPatch>& patches,
	int& outJunctionCount,
	std::string* errMsg)
{
	const int faceCount = static_cast<int>(mesh.faces.size() / 3U);
	if (faceCount < 1)
	{
		if (errMsg)
		{
			*errMsg = "no faces";
		}
		return false;
	}

	std::vector<Vec3d> faceNormals(static_cast<std::size_t>(faceCount));
	std::vector<Vec3d> faceCentroids(static_cast<std::size_t>(faceCount));
	std::vector<std::vector<int>> adj(static_cast<std::size_t>(faceCount));

	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t b = static_cast<std::size_t>(f) * 3U;
		const Vec3d p0 = readV(mesh.vertices, mesh.faces[b]);
		const Vec3d p1 = readV(mesh.vertices, mesh.faces[b + 1U]);
		const Vec3d p2 = readV(mesh.vertices, mesh.faces[b + 2U]);
		faceCentroids[static_cast<std::size_t>(f)] = (p0 + p1 + p2) * (1.0 / 3.0);
		faceNormals[static_cast<std::size_t>(f)] = crossv(p1 - p0, p2 - p0).normalized();
	}

	auto shareEdge = [&](int a, int b) {
		int shared = 0;
		for (int i = 0; i < 3; ++i)
		{
			const int va = mesh.faces[static_cast<std::size_t>(a) * 3U + static_cast<std::size_t>(i)];
			for (int j = 0; j < 3; ++j)
			{
				const int vb = mesh.faces[static_cast<std::size_t>(b) * 3U + static_cast<std::size_t>(j)];
				if (va == vb)
				{
					++shared;
				}
			}
		}
		return shared >= 2;
	};

	for (int a = 0; a < faceCount; ++a)
	{
		for (int b = a + 1; b < faceCount; ++b)
		{
			if (!shareEdge(a, b))
			{
				continue;
			}
			const double ang = std::acos(std::max(
				-1.0,
				std::min(1.0, faceNormals[static_cast<std::size_t>(a)].dot(faceNormals[static_cast<std::size_t>(b)]))));
			if (ang < 1.2)
			{
				adj[static_cast<std::size_t>(a)].push_back(b);
				adj[static_cast<std::size_t>(b)].push_back(a);
			}
		}
	}

	int targetPatches = params.patchCountHint;
	if (targetPatches <= 0)
	{
		targetPatches = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(faceCount) / 80.0)));
	}
	targetPatches = std::min(targetPatches, faceCount);
	targetPatches = std::min(targetPatches, 6);
	const int maxFacesPerPatch = std::max(100, (faceCount + targetPatches - 1) / targetPatches);

	std::vector<int> chart(static_cast<std::size_t>(faceCount), -1);
	int nextChart = 0;
	for (int seed = 0; seed < faceCount && nextChart < targetPatches; ++seed)
	{
		if (chart[static_cast<std::size_t>(seed)] >= 0)
		{
			continue;
		}
		std::queue<int> q;
		q.push(seed);
		chart[static_cast<std::size_t>(seed)] = nextChart;
		int grown = 1;
		while (!q.empty() && grown < maxFacesPerPatch)
		{
			const int f = q.front();
			q.pop();
			for (const int nb : adj[static_cast<std::size_t>(f)])
			{
				if (chart[static_cast<std::size_t>(nb)] < 0)
				{
					chart[static_cast<std::size_t>(nb)] = nextChart;
					q.push(nb);
					++grown;
					if (grown >= maxFacesPerPatch)
					{
						break;
					}
				}
			}
		}
		++nextChart;
	}
	std::vector<int> chartSizes(static_cast<std::size_t>(nextChart), 0);
	for (int f = 0; f < faceCount; ++f)
	{
		if (chart[static_cast<std::size_t>(f)] >= 0)
		{
			++chartSizes[static_cast<std::size_t>(chart[static_cast<std::size_t>(f)])];
		}
	}
	for (int f = 0; f < faceCount; ++f)
	{
		if (chart[static_cast<std::size_t>(f)] >= 0)
		{
			continue;
		}
		for (const int nb : adj[static_cast<std::size_t>(f)])
		{
			if (chart[static_cast<std::size_t>(nb)] >= 0)
			{
				chart[static_cast<std::size_t>(f)] = chart[static_cast<std::size_t>(nb)];
				++chartSizes[static_cast<std::size_t>(chart[static_cast<std::size_t>(f)])];
				break;
			}
		}
		if (chart[static_cast<std::size_t>(f)] >= 0)
		{
			continue;
		}
		int smallestChart = 0;
		for (int c = 1; c < nextChart; ++c)
		{
			if (chartSizes[static_cast<std::size_t>(c)] < chartSizes[static_cast<std::size_t>(smallestChart)])
			{
				smallestChart = c;
			}
		}
		chart[static_cast<std::size_t>(f)] = smallestChart;
		++chartSizes[static_cast<std::size_t>(smallestChart)];
	}

	std::vector<std::vector<int>> chartFaces(static_cast<std::size_t>(nextChart));
	for (int f = 0; f < faceCount; ++f)
	{
		chartFaces[static_cast<std::size_t>(chart[static_cast<std::size_t>(f)])].push_back(f);
	}

	std::vector<int> chartRemap(static_cast<std::size_t>(nextChart), -1);
	patches.clear();
	for (int c = 0; c < nextChart; ++c)
	{
		if (chartFaces[static_cast<std::size_t>(c)].empty())
		{
			continue;
		}
		chartRemap[static_cast<std::size_t>(c)] = static_cast<int>(patches.size());
		QuadPatch patch;
		patch.faceIndices = std::move(chartFaces[static_cast<std::size_t>(c)]);
		patches.push_back(std::move(patch));
	}
	if (patches.empty())
	{
		if (errMsg)
		{
			*errMsg = "partition produced no patches";
		}
		return false;
	}

	mergeTinyPatches(patches, faceCount);
	rebuildPatchAdjacency(adj, faceCount, patches);

	outJunctionCount = 0;
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

	return !patches.empty();
}

} // namespace meshrecon
} // namespace geoalgo
