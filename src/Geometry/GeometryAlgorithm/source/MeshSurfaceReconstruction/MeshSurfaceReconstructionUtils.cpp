/// @file MeshSurfaceReconstructionUtils.cpp
/// @brief PCA 最小特征值 / 最大特征值，衡量平面性（1=完美平面，0=各向同性）

#include "MeshSurfaceReconstructionAmrtoPartition.h"
#include "MeshSurfaceReconstructionInternal.h"
#include "MeshSurfaceReconstructionPartitionCgal.h"

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
bool partitionQuadDomainsHybrid(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
								std::vector<QuadPatch>& patches, int& outJunctionCount,
								MeshSurfaceReconstructReport* partitionStats, std::string* errMsg);

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
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3d readV(const std::vector<float>& v, int i)
{
	const std::size_t b = static_cast<std::size_t>(i) * 3U;
	return {v[b], v[b + 1U], v[b + 2U]};
}

void rebuildPatchAdjacency(const std::vector<std::vector<int>>& adj, const int faceCount,
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

int64_t edgeKey(int v0, int v1)
{
	const int lo = std::min(v0, v1);
	const int hi = std::max(v0, v1);
	return (static_cast<int64_t>(lo) << 32) | static_cast<int64_t>(hi);
}

void collectPatchPts(const IndexedMeshLite& mesh, const std::vector<int>& faceIndices, std::vector<Vec3d>& outPts)
{
	outPts.clear();
	outPts.reserve(faceIndices.size() * 3U);
	for (const int fi : faceIndices)
	{
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		outPts.push_back(readV(mesh.vertices, mesh.faces[b]));
		outPts.push_back(readV(mesh.vertices, mesh.faces[b + 1U]));
		outPts.push_back(readV(mesh.vertices, mesh.faces[b + 2U]));
	}
}

/// PCA 最小特征值 / 最大特征值，衡量平面性（1=完美平面，0=各向同性）
double patchFlatness(const std::vector<Vec3d>& pts)
{
	if (pts.size() < 3U)
	{
		return 1.0;
	}
	Vec3d mean{0, 0, 0};
	for (const auto& p : pts)
	{
		mean = mean + p;
	}
	mean = mean * (1.0 / static_cast<double>(pts.size()));

	double cxx = 0, cyy = 0, czz = 0, cxy = 0, cxz = 0, cyz = 0;
	for (const auto& p : pts)
	{
		const Vec3d d = p - mean;
		cxx += d.x * d.x;
		cyy += d.y * d.y;
		czz += d.z * d.z;
		cxy += d.x * d.y;
		cxz += d.x * d.z;
		cyz += d.y * d.z;
	}

	// 幂迭代近似最大特征值
	double vx = 1.0, vy = 0.5, vz = 0.3;
	for (int iter = 0; iter < 20; ++iter)
	{
		const double nx = cxx * vx + cxy * vy + cxz * vz;
		const double ny = cxy * vx + cyy * vy + cyz * vz;
		const double nz = cxz * vx + cyz * vy + czz * vz;
		const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
		if (len < 1e-20)
		{
			break;
		}
		vx = nx / len;
		vy = ny / len;
		vz = nz / len;
	}
	const double lambdaMax =
		cxx * vx * vx + cyy * vy * vy + czz * vz * vz + 2.0 * cxy * vx * vy + 2.0 * cxz * vx * vz + 2.0 * cyz * vy * vz;

	// trace = lambda0 + lambda1 + lambda2
	const double trace = cxx + cyy + czz;
	// lambdaMin 近似 = (trace - lambdaMax) / 2（假设中间特征值 ≈ lambdaMin）
	const double lambdaMin = std::max(0.0, (trace - lambdaMax) * 0.5);

	if (lambdaMax < 1e-12)
	{
		return 1.0;
	}
	return 1.0 - lambdaMin / lambdaMax;
}

double patchAspectScore(const std::vector<Vec3d>& pts)
{
	if (pts.size() < 3U)
	{
		return 1.0;
	}
	double mnx = 1e30, mny = 1e30, mnz = 1e30;
	double mxx = -1e30, mxy = -1e30, mxz = -1e30;
	for (const auto& p : pts)
	{
		mnx = std::min(mnx, p.x);
		mny = std::min(mny, p.y);
		mnz = std::min(mnz, p.z);
		mxx = std::max(mxx, p.x);
		mxy = std::max(mxy, p.y);
		mxz = std::max(mxz, p.z);
	}
	const double dx = mxx - mnx;
	const double dy = mxy - mny;
	const double dz = mxz - mnz;
	const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
	if (diag < 1e-12)
	{
		return 1.0;
	}
	const double minSide = std::max(1e-12, std::min({dx, dy, dz}));
	return minSide / diag;
}

Vec3d averageFaceNormal(const std::vector<int>& faces, const std::vector<Vec3d>& faceNormals)
{
	Vec3d sum{0, 0, 0};
	for (const int f : faces)
	{
		sum = sum + faceNormals[static_cast<std::size_t>(f)];
	}
	return sum.normalized();
}

double normalSimilarity(const std::vector<int>& facesA, const std::vector<int>& facesB,
						const std::vector<Vec3d>& faceNormals)
{
	const Vec3d na = averageFaceNormal(facesA, faceNormals);
	const Vec3d nb = averageFaceNormal(facesB, faceNormals);
	return std::max(0.0, na.dot(nb));
}

void smoothFaceNormalsForPartition(const std::vector<std::vector<int>>& fullAdj, std::vector<Vec3d>& faceNormals,
								   const int iterations)
{
	if (iterations <= 0)
	{
		return;
	}
	std::vector<Vec3d> tmp = faceNormals;
	for (int iter = 0; iter < iterations; ++iter)
	{
		for (int f = 0; f < static_cast<int>(faceNormals.size()); ++f)
		{
			Vec3d sum = faceNormals[static_cast<std::size_t>(f)];
			int count = 1;
			for (const int nb : fullAdj[static_cast<std::size_t>(f)])
			{
				sum = sum + faceNormals[static_cast<std::size_t>(nb)];
				++count;
			}
			tmp[static_cast<std::size_t>(f)] = (sum * (1.0 / static_cast<double>(count))).normalized();
		}
		faceNormals.swap(tmp);
	}
}

double anglePercentile(std::vector<double> angles, const double percentile)
{
	if (angles.empty())
	{
		return 1.2;
	}
	std::sort(angles.begin(), angles.end());
	const double p = std::max(0.0, std::min(1.0, percentile));
	const std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(angles.size() - 1U));
	return angles[idx];
}

std::vector<int> selectFpsSeeds(const std::vector<Vec3d>& faceCentroids, const int targetPatches)
{
	const int faceCount = static_cast<int>(faceCentroids.size());
	std::vector<int> seeds;
	seeds.reserve(static_cast<std::size_t>(targetPatches));

	int bestSeed = 0;
	double bestDist = -1.0;
	for (int f = 0; f < faceCount; ++f)
	{
		const double d = faceCentroids[static_cast<std::size_t>(f)].length();
		if (d > bestDist)
		{
			bestDist = d;
			bestSeed = f;
		}
	}
	seeds.push_back(bestSeed);

	std::vector<double> minDist(static_cast<std::size_t>(faceCount), 1e30);
	for (int iter = 1; iter < targetPatches; ++iter)
	{
		const int lastSeed = seeds.back();
		for (int f = 0; f < faceCount; ++f)
		{
			const double d =
				(faceCentroids[static_cast<std::size_t>(f)] - faceCentroids[static_cast<std::size_t>(lastSeed)])
					.length();
			minDist[static_cast<std::size_t>(f)] = std::min(minDist[static_cast<std::size_t>(f)], d);
		}
		int nextSeed = 0;
		double maxMinDist = -1.0;
		for (int f = 0; f < faceCount; ++f)
		{
			if (minDist[static_cast<std::size_t>(f)] > maxMinDist)
			{
				maxMinDist = minDist[static_cast<std::size_t>(f)];
				nextSeed = f;
			}
		}
		if (maxMinDist < 1e-9)
		{
			break;
		}
		seeds.push_back(nextSeed);
	}
	return seeds;
}

std::vector<int> multiSourceGeodesicVoronoi(const std::vector<std::vector<int>>& smoothAdj,
											const std::vector<Vec3d>& faceCentroids, const std::vector<int>& seeds)
{
	const int faceCount = static_cast<int>(faceCentroids.size());
	std::vector<double> dist(static_cast<std::size_t>(faceCount), 1e30);
	std::vector<int> chart(static_cast<std::size_t>(faceCount), -1);

	using QueueItem = std::pair<double, int>;
	std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

	for (std::size_t si = 0; si < seeds.size(); ++si)
	{
		const int seed = seeds[si];
		dist[static_cast<std::size_t>(seed)] = 0.0;
		chart[static_cast<std::size_t>(seed)] = static_cast<int>(si);
		pq.push({0.0, seed});
	}

	while (!pq.empty())
	{
		const QueueItem top = pq.top();
		pq.pop();
		const double d = top.first;
		const int f = top.second;
		if (d > dist[static_cast<std::size_t>(f)] + 1e-12)
		{
			continue;
		}
		const int owner = chart[static_cast<std::size_t>(f)];
		for (const int nb : smoothAdj[static_cast<std::size_t>(f)])
		{
			const double edgeLen =
				(faceCentroids[static_cast<std::size_t>(nb)] - faceCentroids[static_cast<std::size_t>(f)]).length();
			const double nd = d + edgeLen;
			if (nd < dist[static_cast<std::size_t>(nb)] - 1e-12)
			{
				dist[static_cast<std::size_t>(nb)] = nd;
				chart[static_cast<std::size_t>(nb)] = owner;
				pq.push({nd, nb});
			}
			else if (std::abs(nd - dist[static_cast<std::size_t>(nb)]) <= 1e-12 &&
					 chart[static_cast<std::size_t>(nb)] >= 0 && owner < chart[static_cast<std::size_t>(nb)])
			{
				chart[static_cast<std::size_t>(nb)] = owner;
			}
		}
	}
	return chart;
}

void assignOrphanChartFaces(std::vector<int>& chart, const std::vector<std::vector<int>>& fullAdj)
{
	bool changed = true;
	while (changed)
	{
		changed = false;
		for (int f = 0; f < static_cast<int>(chart.size()); ++f)
		{
			if (chart[static_cast<std::size_t>(f)] >= 0)
			{
				continue;
			}
			for (const int nb : fullAdj[static_cast<std::size_t>(f)])
			{
				if (chart[static_cast<std::size_t>(nb)] >= 0)
				{
					chart[static_cast<std::size_t>(f)] = chart[static_cast<std::size_t>(nb)];
					changed = true;
					break;
				}
			}
		}
	}
	for (int f = 0; f < static_cast<int>(chart.size()); ++f)
	{
		if (chart[static_cast<std::size_t>(f)] < 0)
		{
			chart[static_cast<std::size_t>(f)] = 0;
		}
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

std::vector<int> multiSourceGeodesicVoronoiOnSubset(const std::vector<std::vector<int>>& smoothAdj,
													const std::vector<Vec3d>& faceCentroids,
													const std::vector<int>& subsetFaces, const std::vector<int>& seeds)
{
	std::unordered_set<int> subsetSet(subsetFaces.begin(), subsetFaces.end());
	std::unordered_map<int, int> faceToLocal;
	for (std::size_t i = 0; i < subsetFaces.size(); ++i)
	{
		faceToLocal[subsetFaces[i]] = static_cast<int>(i);
	}

	std::vector<double> dist(subsetFaces.size(), 1e30);
	std::vector<int> owner(subsetFaces.size(), -1);

	using QueueItem = std::pair<double, int>;
	std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

	for (std::size_t si = 0; si < seeds.size(); ++si)
	{
		const auto it = faceToLocal.find(seeds[si]);
		if (it == faceToLocal.end())
		{
			continue;
		}
		const int li = it->second;
		dist[static_cast<std::size_t>(li)] = 0.0;
		owner[static_cast<std::size_t>(li)] = static_cast<int>(si);
		pq.push({0.0, seeds[si]});
	}

	while (!pq.empty())
	{
		const QueueItem top = pq.top();
		pq.pop();
		const double d = top.first;
		const int f = top.second;
		const auto lit = faceToLocal.find(f);
		if (lit == faceToLocal.end())
		{
			continue;
		}
		const int li = lit->second;
		if (d > dist[static_cast<std::size_t>(li)] + 1e-12)
		{
			continue;
		}
		const int seedOwner = owner[static_cast<std::size_t>(li)];
		for (const int nb : smoothAdj[static_cast<std::size_t>(f)])
		{
			if (subsetSet.find(nb) == subsetSet.end())
			{
				continue;
			}
			const auto nlit = faceToLocal.find(nb);
			if (nlit == faceToLocal.end())
			{
				continue;
			}
			const int nli = nlit->second;
			const double edgeLen =
				(faceCentroids[static_cast<std::size_t>(nb)] - faceCentroids[static_cast<std::size_t>(f)]).length();
			const double nd = d + edgeLen;
			if (nd < dist[static_cast<std::size_t>(nli)] - 1e-12)
			{
				dist[static_cast<std::size_t>(nli)] = nd;
				owner[static_cast<std::size_t>(nli)] = seedOwner;
				pq.push({nd, nb});
			}
			else if (std::abs(nd - dist[static_cast<std::size_t>(nli)]) <= 1e-12 &&
					 owner[static_cast<std::size_t>(nli)] >= 0 && seedOwner < owner[static_cast<std::size_t>(nli)])
			{
				owner[static_cast<std::size_t>(nli)] = seedOwner;
			}
		}
	}

	std::vector<int> faceOwner(static_cast<std::size_t>(faceCentroids.size()), -1);
	for (std::size_t i = 0; i < subsetFaces.size(); ++i)
	{
		if (owner[i] >= 0)
		{
			faceOwner[static_cast<std::size_t>(subsetFaces[i])] = owner[i];
		}
	}
	return faceOwner;
}

void splitOversizedPatches(std::vector<QuadPatch>& patches, const std::vector<std::vector<int>>& smoothAdj,
						   const std::vector<Vec3d>& faceCentroids, const int maxFacesPerPatch)
{
	const int splitThreshold = static_cast<int>(static_cast<double>(maxFacesPerPatch) * 1.25);
	std::vector<QuadPatch> nextPatches;
	nextPatches.reserve(patches.size() + 4U);

	for (QuadPatch& patch : patches)
	{
		if (static_cast<int>(patch.faceIndices.size()) <= splitThreshold)
		{
			nextPatches.push_back(std::move(patch));
			continue;
		}

		const std::vector<int>& patchFaces = patch.faceIndices;
		std::vector<Vec3d> subsetCentroids;
		subsetCentroids.reserve(patchFaces.size());
		for (const int f : patchFaces)
		{
			subsetCentroids.push_back(faceCentroids[static_cast<std::size_t>(f)]);
		}
		const std::vector<int> subsetSeeds = selectFpsSeeds(subsetCentroids, 2);

		std::vector<int> localSeeds;
		localSeeds.reserve(2U);
		for (const int ls : subsetSeeds)
		{
			if (ls >= 0 && ls < static_cast<int>(patchFaces.size()))
			{
				localSeeds.push_back(patchFaces[static_cast<std::size_t>(ls)]);
			}
		}
		if (localSeeds.size() < 2U)
		{
			nextPatches.push_back(std::move(patch));
			continue;
		}

		const std::vector<int> faceOwner =
			multiSourceGeodesicVoronoiOnSubset(smoothAdj, faceCentroids, patch.faceIndices, localSeeds);

		std::vector<std::vector<int>> splitFaces(2U);
		for (const int f : patch.faceIndices)
		{
			const int o = faceOwner[static_cast<std::size_t>(f)];
			const int bucket = (o <= 0) ? 0 : 1;
			splitFaces[static_cast<std::size_t>(bucket)].push_back(f);
		}
		if (splitFaces[0].empty() || splitFaces[1].empty())
		{
			nextPatches.push_back(std::move(patch));
			continue;
		}
		for (std::size_t si = 0; si < splitFaces.size(); ++si)
		{
			QuadPatch child;
			child.faceIndices = std::move(splitFaces[si]);
			nextPatches.push_back(std::move(child));
		}
	}
	patches = std::move(nextPatches);
}

void cleanupDisconnectedComponents(std::vector<QuadPatch>& patches, const std::vector<std::vector<int>>& fullAdj,
								   const std::vector<Vec3d>& faceNormals, const int minFaces)
{
	if (patches.size() <= 1U)
	{
		return;
	}

	std::vector<int> faceToPatch(static_cast<std::size_t>(fullAdj.size()), -1);
	for (int pi = 0; pi < static_cast<int>(patches.size()); ++pi)
	{
		for (const int f : patches[static_cast<std::size_t>(pi)].faceIndices)
		{
			faceToPatch[static_cast<std::size_t>(f)] = pi;
		}
	}

	bool changed = true;
	while (changed)
	{
		changed = false;
		faceToPatch.assign(fullAdj.size(), -1);
		for (int pi = 0; pi < static_cast<int>(patches.size()); ++pi)
		{
			for (const int f : patches[static_cast<std::size_t>(pi)].faceIndices)
			{
				faceToPatch[static_cast<std::size_t>(f)] = pi;
			}
		}

		for (int pi = 0; pi < static_cast<int>(patches.size()); ++pi)
		{
			auto& faces = patches[static_cast<std::size_t>(pi)].faceIndices;
			if (faces.empty())
			{
				continue;
			}
			std::unordered_set<int> faceSet(faces.begin(), faces.end());
			std::unordered_set<int> visited;
			std::vector<std::vector<int>> components;

			for (const int start : faces)
			{
				if (visited.find(start) != visited.end())
				{
					continue;
				}
				std::vector<int> comp;
				std::queue<int> q;
				q.push(start);
				visited.insert(start);
				while (!q.empty())
				{
					const int f = q.front();
					q.pop();
					comp.push_back(f);
					for (const int nb : fullAdj[static_cast<std::size_t>(f)])
					{
						if (faceSet.find(nb) != faceSet.end() && visited.find(nb) == visited.end())
						{
							visited.insert(nb);
							q.push(nb);
						}
					}
				}
				components.push_back(std::move(comp));
			}

			if (components.size() <= 1U)
			{
				continue;
			}

			int largestIdx = 0;
			for (std::size_t ci = 1; ci < components.size(); ++ci)
			{
				if (components[ci].size() > components[largestIdx].size())
				{
					largestIdx = static_cast<int>(ci);
				}
			}

			std::vector<int> kept = std::move(components[static_cast<std::size_t>(largestIdx)]);
			for (std::size_t ci = 0; ci < components.size(); ++ci)
			{
				if (static_cast<int>(ci) == largestIdx)
				{
					continue;
				}
				auto& comp = components[ci];
				if (static_cast<int>(comp.size()) >= minFaces)
				{
					QuadPatch orphan;
					orphan.faceIndices = std::move(comp);
					patches.push_back(std::move(orphan));
					continue;
				}

				std::unordered_set<int> neighborPatches;
				for (const int f : comp)
				{
					for (const int nb : fullAdj[static_cast<std::size_t>(f)])
					{
						const int cp = faceToPatch[static_cast<std::size_t>(nb)];
						if (cp >= 0 && cp != pi)
						{
							neighborPatches.insert(cp);
						}
					}
				}

				int bestNeighbor = -1;
				double bestSim = -1.0;
				for (const int ni : neighborPatches)
				{
					const double sim =
						normalSimilarity(comp, patches[static_cast<std::size_t>(ni)].faceIndices, faceNormals);
					if (sim > bestSim)
					{
						bestSim = sim;
						bestNeighbor = ni;
					}
				}
				if (bestNeighbor >= 0)
				{
					auto& dst = patches[static_cast<std::size_t>(bestNeighbor)].faceIndices;
					dst.insert(dst.end(), comp.begin(), comp.end());
					changed = true;
				}
				else
				{
					kept.insert(kept.end(), comp.begin(), comp.end());
				}
			}
			faces = std::move(kept);
		}
	}

	patches.erase(
		std::remove_if(patches.begin(), patches.end(), [](const QuadPatch& p) { return p.faceIndices.empty(); }),
		patches.end());
}

void mergeSmallPatches(std::vector<QuadPatch>& patches, const std::vector<std::vector<int>>& fullAdj,
					   const IndexedMeshLite& mesh, const std::vector<Vec3d>& faceNormals, const int faceCount,
					   const int targetPatches, const MeshSurfaceReconstructParams& params)
{
	const bool useQuadScore =
		params.partitionMode == MeshSurfacePartitionMode::CgalChartHybrid || params.sdfSeedBlendWeight > 1e-6;
	const int minFaces = std::max(100, faceCount / std::max(1, targetPatches * 2));
	const int targetAvg = std::max(minFaces, faceCount / std::max(1, targetPatches));
	const int forceMergeBelow = std::max(1, targetAvg / 4);

	if (patches.size() <= 1U)
	{
		return;
	}

	std::vector<int> faceToPatch(static_cast<std::size_t>(faceCount), -1);

	auto rebuildFaceToPatch = [&]()
	{
		faceToPatch.assign(static_cast<std::size_t>(faceCount), -1);
		for (int pi = 0; pi < static_cast<int>(patches.size()); ++pi)
		{
			for (const int f : patches[static_cast<std::size_t>(pi)].faceIndices)
			{
				faceToPatch[static_cast<std::size_t>(f)] = pi;
			}
		}
	};

	auto tryMergeSmallest = [&](const int sizeThreshold) -> bool
	{
		rebuildFaceToPatch();
		int smallestIdx = -1;
		int smallestSize = std::numeric_limits<int>::max();
		for (int pi = 0; pi < static_cast<int>(patches.size()); ++pi)
		{
			const int sz = static_cast<int>(patches[static_cast<std::size_t>(pi)].faceIndices.size());
			if (sz > 0 && sz < smallestSize)
			{
				smallestSize = sz;
				smallestIdx = pi;
			}
		}
		if (smallestIdx < 0 || smallestSize >= sizeThreshold)
		{
			return false;
		}

		const auto& smallFaces = patches[static_cast<std::size_t>(smallestIdx)].faceIndices;
		std::unordered_set<int> neighborPatches;
		for (const int f : smallFaces)
		{
			for (const int nb : fullAdj[static_cast<std::size_t>(f)])
			{
				const int cp = faceToPatch[static_cast<std::size_t>(nb)];
				if (cp >= 0 && cp != smallestIdx)
				{
					neighborPatches.insert(cp);
				}
			}
		}

		int bestNeighbor = -1;
		double bestScore = -1.0;
		for (const int ni : neighborPatches)
		{
			std::vector<int> mergedFaces = patches[static_cast<std::size_t>(ni)].faceIndices;
			mergedFaces.insert(mergedFaces.end(), smallFaces.begin(), smallFaces.end());
			std::vector<Vec3d> mergedPts;
			collectPatchPts(mesh, mergedFaces, mergedPts);
			const double flat = patchFlatness(mergedPts);
			const double aspect = patchAspectScore(mergedPts);
			const double normalSim =
				normalSimilarity(smallFaces, patches[static_cast<std::size_t>(ni)].faceIndices, faceNormals);
			double quadScore = 1.0;
			if (useQuadScore)
			{
				QuadPatch probe;
				probe.faceIndices = mergedFaces;
				assignPatchCornerMetadata(mesh, probe);
				quadScore = probe.hasSquareCorners ? 1.0 : 0.35;
			}
			const double score = useQuadScore ? (0.4 * flat + 0.15 * aspect + 0.25 * normalSim + 0.2 * quadScore)
											  : (0.5 * flat + 0.2 * aspect + 0.3 * normalSim);
			if (score > bestScore)
			{
				bestScore = score;
				bestNeighbor = ni;
			}
		}

		if (bestNeighbor >= 0)
		{
			auto& dst = patches[static_cast<std::size_t>(bestNeighbor)].faceIndices;
			dst.insert(dst.end(), smallFaces.begin(), smallFaces.end());
			patches[static_cast<std::size_t>(smallestIdx)].faceIndices.clear();
			return true;
		}
		return false;
	};

	bool changed = true;
	while (changed)
	{
		changed = tryMergeSmallest(minFaces);
	}

	changed = true;
	while (changed)
	{
		changed = tryMergeSmallest(forceMergeBelow);
	}

	patches.erase(
		std::remove_if(patches.begin(), patches.end(), [](const QuadPatch& p) { return p.faceIndices.empty(); }),
		patches.end());
}

void computePatchFaceStats(const std::vector<QuadPatch>& patches, const int minFacesThreshold, int& outMin, int& outMax,
						   int& outSmallCount)
{
	outMin = 0;
	outMax = 0;
	outSmallCount = 0;
	if (patches.empty())
	{
		return;
	}
	outMin = std::numeric_limits<int>::max();
	outMax = 0;
	for (const QuadPatch& patch : patches)
	{
		const int sz = static_cast<int>(patch.faceIndices.size());
		outMin = std::min(outMin, sz);
		outMax = std::max(outMax, sz);
		if (sz < minFacesThreshold)
		{
			++outSmallCount;
		}
	}
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
		std::size_t operator()(const Key& k) const { return static_cast<std::size_t>(k.x ^ (k.y << 16) ^ (k.z << 32)); }
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
			Key key{static_cast<int64_t>(std::round(soup[b] * scale)),
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

static bool partitionQuadDomainsGeodesicV3(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
										   std::vector<QuadPatch>& patches, int& outJunctionCount,
										   MeshSurfaceReconstructReport* partitionStats, std::string* errMsg);

bool partitionQuadDomains(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
						  std::vector<QuadPatch>& patches, int& outJunctionCount,
						  MeshSurfaceReconstructReport* partitionStats, std::string* errMsg)
{
	if (params.partitionMode == MeshSurfacePartitionMode::HybridNormalCvt)
	{
		const bool ok = partitionQuadDomainsHybrid(mesh, params, patches, outJunctionCount, partitionStats, errMsg);
		if (ok)
		{
			assignAllPatchCornerMetadata(mesh, patches);
		}
		return ok;
	}
	if (params.partitionMode == MeshSurfacePartitionMode::CgalChartHybrid)
	{
		return partitionQuadDomainsCgalChartHybrid(mesh, params, patches, outJunctionCount, partitionStats, errMsg);
	}
	if (params.partitionMode == MeshSurfacePartitionMode::AmrtoImGmcg)
	{
		return partitionQuadDomainsAmrtoImGmcg(mesh, params, patches, outJunctionCount, partitionStats, errMsg, nullptr,
											   nullptr);
	}
	const bool ok = partitionQuadDomainsGeodesicV3(mesh, params, patches, outJunctionCount, partitionStats, errMsg);
	if (ok)
	{
		assignAllPatchCornerMetadata(mesh, patches);
	}
	return ok;
}

static bool partitionQuadDomainsGeodesicV3(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
										   std::vector<QuadPatch>& patches, int& outJunctionCount,
										   MeshSurfaceReconstructReport* partitionStats, std::string* errMsg)
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

	std::vector<Vec3d> rawFaceNormals(static_cast<std::size_t>(faceCount));
	std::vector<Vec3d> faceNormals(static_cast<std::size_t>(faceCount));
	std::vector<Vec3d> faceCentroids(static_cast<std::size_t>(faceCount));
	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t b = static_cast<std::size_t>(f) * 3U;
		const Vec3d p0 = readV(mesh.vertices, mesh.faces[b]);
		const Vec3d p1 = readV(mesh.vertices, mesh.faces[b + 1U]);
		const Vec3d p2 = readV(mesh.vertices, mesh.faces[b + 2U]);
		faceCentroids[static_cast<std::size_t>(f)] = (p0 + p1 + p2) * (1.0 / 3.0);
		const Vec3d n = crossv(p1 - p0, p2 - p0).normalized();
		rawFaceNormals[static_cast<std::size_t>(f)] = n;
		faceNormals[static_cast<std::size_t>(f)] = n;
	}

	struct EdgeFaces
	{
		int f0 = -1;
		int f1 = -1;
	};
	std::unordered_map<int64_t, EdgeFaces> edgeToFaces;
	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t b = static_cast<std::size_t>(f) * 3U;
		for (int e = 0; e < 3; ++e)
		{
			const int v0 = mesh.faces[b + static_cast<std::size_t>(e)];
			const int v1 = mesh.faces[b + static_cast<std::size_t>((e + 1) % 3)];
			const int64_t key = edgeKey(v0, v1);
			auto& ef = edgeToFaces[key];
			if (ef.f0 < 0)
			{
				ef.f0 = f;
			}
			else if (ef.f1 < 0)
			{
				ef.f1 = f;
			}
		}
	}

	std::vector<std::vector<int>> fullAdj(static_cast<std::size_t>(faceCount));
	std::vector<std::vector<int>> smoothAdj(static_cast<std::size_t>(faceCount));
	std::vector<double> allAngles;
	allAngles.reserve(edgeToFaces.size());

	for (const auto& kv : edgeToFaces)
	{
		const EdgeFaces& ef = kv.second;
		if (ef.f0 < 0 || ef.f1 < 0)
		{
			continue;
		}
		fullAdj[static_cast<std::size_t>(ef.f0)].push_back(ef.f1);
		fullAdj[static_cast<std::size_t>(ef.f1)].push_back(ef.f0);
	}

	smoothFaceNormalsForPartition(fullAdj, faceNormals, params.partitionNormalSmoothIters);

	for (const auto& kv : edgeToFaces)
	{
		const EdgeFaces& ef = kv.second;
		if (ef.f0 < 0 || ef.f1 < 0)
		{
			continue;
		}
		const double dotVal = std::max(-1.0, std::min(1.0, faceNormals[static_cast<std::size_t>(ef.f0)].dot(
															   faceNormals[static_cast<std::size_t>(ef.f1)])));
		const double angle = std::acos(dotVal);
		allAngles.push_back(angle);
	}

	double featureAngleThreshold = params.featureThresholdC0;
	if (featureAngleThreshold <= 0.0 || featureAngleThreshold > 3.14)
	{
		featureAngleThreshold = 1.2;
	}
	if (!allAngles.empty())
	{
		std::vector<double> sortedAngles = allAngles;
		std::sort(sortedAngles.begin(), sortedAngles.end());
		const double median = sortedAngles[sortedAngles.size() / 2U];
		const double percentile = std::max(0.5, std::min(0.99, params.featureAnglePercentile));
		const double pctAngle = anglePercentile(allAngles, percentile);
		const double lowerBound = median * 0.5;
		const double upperBound = std::min(3.14, median * 3.0);
		featureAngleThreshold = std::max(featureAngleThreshold, pctAngle);
		featureAngleThreshold = std::max(lowerBound, std::min(upperBound, featureAngleThreshold));
	}

	for (const auto& kv : edgeToFaces)
	{
		const EdgeFaces& ef = kv.second;
		if (ef.f0 < 0 || ef.f1 < 0)
		{
			continue;
		}
		const double dotVal = std::max(-1.0, std::min(1.0, faceNormals[static_cast<std::size_t>(ef.f0)].dot(
															   faceNormals[static_cast<std::size_t>(ef.f1)])));
		const double angle = std::acos(dotVal);
		if (angle < featureAngleThreshold)
		{
			smoothAdj[static_cast<std::size_t>(ef.f0)].push_back(ef.f1);
			smoothAdj[static_cast<std::size_t>(ef.f1)].push_back(ef.f0);
		}
	}

	int targetPatches = params.patchCountHint;
	if (targetPatches <= 0)
	{
		targetPatches = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(faceCount) / 80.0)));
	}
	targetPatches = std::min(targetPatches, faceCount);
	const int maxFacesPerPatch = std::max(100, (faceCount + targetPatches - 1) / targetPatches);
	const int minFacesThreshold = std::max(100, faceCount / std::max(1, targetPatches * 2));

	const std::vector<int> fpsSeeds = selectFpsSeeds(faceCentroids, targetPatches);
	std::vector<int> seeds = fpsSeeds;
	if (params.partitionMode == MeshSurfacePartitionMode::CgalChartHybrid || params.sdfSeedBlendWeight > 1e-6)
	{
		std::vector<int> sdfSeeds;
		const int segCount = params.sdfSegmentCount > 0 ? params.sdfSegmentCount : std::max(4, targetPatches);
		if (collectSdfSegmentSeedFaces(mesh, segCount, sdfSeeds, nullptr))
		{
			for (const int sf : sdfSeeds)
			{
				if (std::find(seeds.begin(), seeds.end(), sf) == seeds.end())
				{
					seeds.push_back(sf);
				}
			}
		}
	}
	std::vector<int> chart = multiSourceGeodesicVoronoi(smoothAdj, faceCentroids, seeds);
	assignOrphanChartFaces(chart, fullAdj);
	chartToPatches(chart, patches);
	if (patches.empty())
	{
		if (errMsg)
		{
			*errMsg = "partition produced no patches";
		}
		return false;
	}

	int preMergeMin = 0;
	int preMergeMax = 0;
	int preMergeSmall = 0;
	computePatchFaceStats(patches, minFacesThreshold, preMergeMin, preMergeMax, preMergeSmall);
	if (partitionStats)
	{
		partitionStats->smallPatchCount = preMergeSmall;
	}

	splitOversizedPatches(patches, smoothAdj, faceCentroids, maxFacesPerPatch);
	cleanupDisconnectedComponents(patches, fullAdj, rawFaceNormals, minFacesThreshold);
	mergeSmallPatches(patches, fullAdj, mesh, rawFaceNormals, faceCount, targetPatches, params);

	if (patches.empty())
	{
		if (errMsg)
		{
			*errMsg = "partition produced no patches";
		}
		return false;
	}

	int finalMin = 0;
	int finalMax = 0;
	int finalSmall = 0;
	computePatchFaceStats(patches, minFacesThreshold, finalMin, finalMax, finalSmall);
	if (partitionStats)
	{
		partitionStats->minFacesPerPatch = finalMin;
		partitionStats->maxFacesPerPatch = finalMax;
	}

	rebuildPatchAdjacency(fullAdj, faceCount, patches);

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
