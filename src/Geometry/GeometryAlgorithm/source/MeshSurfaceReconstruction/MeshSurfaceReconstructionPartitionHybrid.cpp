/// @file MeshSurfaceReconstructionPartitionHybrid.cpp
/// @brief MeshSurfaceReconstructionPartitionHybrid 实现

#include "MeshSurfaceReconstructionPartitionCgal.h"
#include "MeshSurfaceReconstructionPartitionCommon.h"
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
struct GeneratorCluster
{
	PartitionVec3d c;
	int count = 0;
};

enum class ClusterMetric
{
	NormalDot,
	EuclideanDist,
};

int computeNthresh(const MeshSurfaceReconstructParams& params, const int faceCount)
{
	const double raw = params.hybridSmallRegionRatio * static_cast<double>(faceCount);
	int n = static_cast<int>(std::round(raw));
	n = std::max(params.hybridSmallRegionMin, std::min(params.hybridSmallRegionMax, n));
	return std::max(1, n);
}

PartitionVec3d regionAverageNormal(const std::vector<int>& faces, const std::vector<PartitionVec3d>& faceNormals)
{
	PartitionVec3d sum{0, 0, 0};
	for (const int f : faces)
	{
		sum = sum + faceNormals[static_cast<std::size_t>(f)];
	}
	return sum.normalized();
}

bool runAlgorithm1(const std::vector<int>& activeFaces, const std::vector<PartitionVec3d>& attrs,
				   std::vector<GeneratorCluster>& generators, std::vector<int>& labels, const ClusterMetric metric,
				   const int maxIters)
{
	if (activeFaces.empty() || generators.empty())
	{
		return false;
	}
	std::vector<int> faceToActiveIndex(static_cast<std::size_t>(attrs.size()), -1);
	for (std::size_t i = 0; i < activeFaces.size(); ++i)
	{
		faceToActiveIndex[static_cast<std::size_t>(activeFaces[i])] = static_cast<int>(i);
	}

	labels.assign(activeFaces.size(), 0);
	for (auto& g : generators)
	{
		g.count = 0;
	}
	for (std::size_t i = 0; i < activeFaces.size(); ++i)
	{
		const int f = activeFaces[i];
		int best = 0;
		double bestScore = -1e30;
		for (int gi = 0; gi < static_cast<int>(generators.size()); ++gi)
		{
			double score = 0.0;
			if (metric == ClusterMetric::NormalDot)
			{
				score = attrs[static_cast<std::size_t>(f)].dot(generators[static_cast<std::size_t>(gi)].c.normalized());
			}
			else
			{
				score = -(attrs[static_cast<std::size_t>(f)] - generators[static_cast<std::size_t>(gi)].c).length();
			}
			if (score > bestScore)
			{
				bestScore = score;
				best = gi;
			}
		}
		labels[i] = best;
	}

	bool changed = true;
	for (int iter = 0; iter < maxIters && changed; ++iter)
	{
		changed = false;
		for (auto& g : generators)
		{
			g.c = PartitionVec3d{0, 0, 0};
			g.count = 0;
		}
		for (std::size_t i = 0; i < activeFaces.size(); ++i)
		{
			const int gi = labels[i];
			const int f = activeFaces[i];
			generators[static_cast<std::size_t>(gi)].c =
				generators[static_cast<std::size_t>(gi)].c + attrs[static_cast<std::size_t>(f)];
			++generators[static_cast<std::size_t>(gi)].count;
		}
		for (auto& g : generators)
		{
			if (g.count > 0)
			{
				g.c = g.c * (1.0 / static_cast<double>(g.count));
			}
		}

		for (std::size_t i = 0; i < activeFaces.size(); ++i)
		{
			const int f = activeFaces[i];
			int best = labels[i];
			double bestScore = -1e30;
			for (int gi = 0; gi < static_cast<int>(generators.size()); ++gi)
			{
				double score = 0.0;
				if (metric == ClusterMetric::NormalDot)
				{
					score =
						attrs[static_cast<std::size_t>(f)].dot(generators[static_cast<std::size_t>(gi)].c.normalized());
				}
				else
				{
					score = -(attrs[static_cast<std::size_t>(f)] - generators[static_cast<std::size_t>(gi)].c).length();
				}
				if (score > bestScore)
				{
					bestScore = score;
					best = gi;
				}
			}
			if (best != labels[i])
			{
				labels[i] = best;
				changed = true;
			}
		}
	}
	return true;
}

std::vector<std::vector<int>> extractConnectedRegions(const std::vector<int>& faceCluster,
													  const std::vector<std::vector<int>>& fullAdj)
{
	const int faceCount = static_cast<int>(faceCluster.size());
	std::vector<char> visited(static_cast<std::size_t>(faceCount), 0);
	std::vector<std::vector<int>> regions;
	for (int f = 0; f < faceCount; ++f)
	{
		if (visited[static_cast<std::size_t>(f)] != 0 || faceCluster[static_cast<std::size_t>(f)] < 0)
		{
			continue;
		}
		const int clusterId = faceCluster[static_cast<std::size_t>(f)];
		std::vector<int> region;
		std::queue<int> q;
		q.push(f);
		visited[static_cast<std::size_t>(f)] = 1;
		while (!q.empty())
		{
			const int cur = q.front();
			q.pop();
			region.push_back(cur);
			for (const int nb : fullAdj[static_cast<std::size_t>(cur)])
			{
				if (visited[static_cast<std::size_t>(nb)] != 0)
				{
					continue;
				}
				if (faceCluster[static_cast<std::size_t>(nb)] == clusterId)
				{
					visited[static_cast<std::size_t>(nb)] = 1;
					q.push(nb);
				}
			}
		}
		if (!region.empty())
		{
			regions.push_back(std::move(region));
		}
	}
	return regions;
}

void cleanupInitialRegions(const MeshSurfaceReconstructParams& params, const int faceCount,
						   const std::vector<std::vector<int>>& fullAdj, const std::vector<PartitionVec3d>& faceNormals,
						   std::vector<std::vector<int>>& regions)
{
	const int nThresh = computeNthresh(params, faceCount);
	bool changed = true;
	while (changed)
	{
		changed = false;
		std::unordered_map<int, std::vector<int>> faceToRegion;
		for (int ri = 0; ri < static_cast<int>(regions.size()); ++ri)
		{
			for (const int f : regions[static_cast<std::size_t>(ri)])
			{
				faceToRegion[f] = {ri};
			}
		}

		int mergeIdx = -1;
		int mergeInto = -1;
		for (int ri = 0; ri < static_cast<int>(regions.size()); ++ri)
		{
			const auto& faces = regions[static_cast<std::size_t>(ri)];
			if (faces.empty())
			{
				continue;
			}
			const PartitionVec3d vi = regionAverageNormal(faces, faceNormals);
			std::unordered_set<int> neighborRegions;
			for (const int f : faces)
			{
				for (const int nb : fullAdj[static_cast<std::size_t>(f)])
				{
					for (int rj = 0; rj < static_cast<int>(regions.size()); ++rj)
					{
						if (rj == ri)
						{
							continue;
						}
						const auto& nf = regions[static_cast<std::size_t>(rj)];
						if (std::find(nf.begin(), nf.end(), nb) != nf.end())
						{
							neighborRegions.insert(rj);
						}
					}
				}
			}
			int bestNeighbor = -1;
			double bestCos = -2.0;
			for (const int rj : neighborRegions)
			{
				const PartitionVec3d vj = regionAverageNormal(regions[static_cast<std::size_t>(rj)], faceNormals);
				const double cosVal = vi.dot(vj);
				if (cosVal > bestCos)
				{
					bestCos = cosVal;
					bestNeighbor = rj;
				}
			}
			if (bestNeighbor < 0)
			{
				continue;
			}
			const int nTri = static_cast<int>(faces.size());
			const double cosHigh = params.hybridMergeCosHigh;
			const double cosLow =
				params.hybridMergeCosLowBase +
				params.hybridMergeCosLowScale * (static_cast<double>(nTri) / static_cast<double>(nThresh));
			if (bestCos > cosHigh || (nTri < nThresh && bestCos > cosLow))
			{
				mergeIdx = ri;
				mergeInto = bestNeighbor;
				break;
			}
		}
		if (mergeIdx >= 0 && mergeInto >= 0)
		{
			auto& dst = regions[static_cast<std::size_t>(mergeInto)];
			const auto& src = regions[static_cast<std::size_t>(mergeIdx)];
			dst.insert(dst.end(), src.begin(), src.end());
			regions[static_cast<std::size_t>(mergeIdx)].clear();
			changed = true;
			regions.erase(
				std::remove_if(regions.begin(), regions.end(), [](const std::vector<int>& r) { return r.empty(); }),
				regions.end());
		}
	}
}

int computeSecondarySampleCount(const MeshSurfaceReconstructParams& params, const int faceCount,
								const int initialRegionCount, const double regionArea, const double totalArea,
								const int patchCountHint)
{
	double s = params.hybridSecondarySampleScale * static_cast<double>(initialRegionCount) * regionArea / totalArea;
	if (patchCountHint > 0)
	{
		const int autoHint = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(faceCount) / 80.0)));
		s *= static_cast<double>(patchCountHint) / static_cast<double>(autoHint);
	}
	if (s < 3.0)
	{
		return std::max(1, static_cast<int>(std::round(s)));
	}
	return std::max(1, static_cast<int>(std::round(s / std::log(s))));
}

std::vector<int> pickEquispacedFaces(const std::vector<int>& regionFaces, const int sampleCount)
{
	if (regionFaces.empty())
	{
		return {};
	}
	if (sampleCount <= 1)
	{
		return {regionFaces[0]};
	}
	std::vector<int> samples;
	samples.reserve(static_cast<std::size_t>(sampleCount));
	const double step = static_cast<double>(regionFaces.size()) / static_cast<double>(sampleCount);
	for (int i = 0; i < sampleCount; ++i)
	{
		const int idx = std::min(static_cast<int>(regionFaces.size()) - 1,
								 static_cast<int>(std::floor((static_cast<double>(i) + 0.5) * step)));
		samples.push_back(regionFaces[static_cast<std::size_t>(idx)]);
	}
	return samples;
}

} // namespace

bool partitionQuadDomainsHybrid(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
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

	const MeshAdjacency adj = buildMeshAdjacency(mesh, faceCount);
	std::vector<PartitionVec3d> faceNormals;
	std::vector<PartitionVec3d> faceCentroids;
	std::vector<double> faceAreas;
	computeFaceGeometry(mesh, faceCount, faceNormals, faceCentroids, faceAreas);

	double totalArea = 0.0;
	for (const double a : faceAreas)
	{
		totalArea += a;
	}
	if (totalArea < 1e-18)
	{
		totalArea = static_cast<double>(faceCount);
	}

	std::vector<GeneratorCluster> axisGenerators(6);
	axisGenerators[0].c = {1, 0, 0};
	axisGenerators[1].c = {-1, 0, 0};
	axisGenerators[2].c = {0, 1, 0};
	axisGenerators[3].c = {0, -1, 0};
	axisGenerators[4].c = {0, 0, 1};
	axisGenerators[5].c = {0, 0, -1};

	if (params.partitionMode == MeshSurfacePartitionMode::CgalChartHybrid || params.sdfSeedBlendWeight > 1e-6)
	{
		std::vector<int> sdfSeeds;
		const int segCount = params.sdfSegmentCount > 0
								 ? params.sdfSegmentCount
								 : std::max(6, params.patchCountHint > 0 ? params.patchCountHint : 8);
		if (collectSdfSegmentSeedFaces(mesh, segCount, sdfSeeds, nullptr))
		{
			for (const int sf : sdfSeeds)
			{
				if (sf < 0 || static_cast<std::size_t>(sf) >= faceCentroids.size())
				{
					continue;
				}
				GeneratorCluster g;
				g.c = faceCentroids[static_cast<std::size_t>(sf)];
				g.count = 1;
				axisGenerators.push_back(g);
			}
		}
	}

	std::vector<int> allFaces(static_cast<std::size_t>(faceCount));
	for (int f = 0; f < faceCount; ++f)
	{
		allFaces[static_cast<std::size_t>(f)] = f;
	}

	std::vector<int> clusterLabels;
	runAlgorithm1(allFaces, faceNormals, axisGenerators, clusterLabels, ClusterMetric::NormalDot,
				  params.hybridClusterMaxIters);

	std::vector<int> faceCluster(static_cast<std::size_t>(faceCount), 0);
	for (std::size_t i = 0; i < allFaces.size(); ++i)
	{
		faceCluster[static_cast<std::size_t>(allFaces[i])] = clusterLabels[i];
	}

	std::vector<std::vector<int>> initialRegions = extractConnectedRegions(faceCluster, adj.fullAdj);
	cleanupInitialRegions(params, faceCount, adj.fullAdj, faceNormals, initialRegions);

	if (partitionStats)
	{
		partitionStats->initialRegionCount = static_cast<int>(initialRegions.size());
	}

	std::vector<int> faceToPatch(static_cast<std::size_t>(faceCount), -1);
	int nextPatchId = 0;
	const int initialRegionCount = static_cast<int>(initialRegions.size());

	for (const std::vector<int>& regionFaces : initialRegions)
	{
		if (regionFaces.empty())
		{
			continue;
		}
		double regionArea = 0.0;
		for (const int f : regionFaces)
		{
			regionArea += faceAreas[static_cast<std::size_t>(f)];
		}
		const int sampleCount = computeSecondarySampleCount(params, faceCount, initialRegionCount, regionArea,
															totalArea, params.patchCountHint);

		if (sampleCount <= 1 || static_cast<int>(regionFaces.size()) <= sampleCount)
		{
			for (const int f : regionFaces)
			{
				faceToPatch[static_cast<std::size_t>(f)] = nextPatchId;
			}
			++nextPatchId;
			continue;
		}

		const std::vector<int> sampleFaceIds = pickEquispacedFaces(regionFaces, sampleCount);
		std::vector<GeneratorCluster> sampleGenerators;
		sampleGenerators.reserve(sampleFaceIds.size());
		for (const int sf : sampleFaceIds)
		{
			GeneratorCluster g;
			g.c = faceCentroids[static_cast<std::size_t>(sf)];
			g.count = 1;
			sampleGenerators.push_back(g);
		}

		std::vector<int> subLabels;
		runAlgorithm1(regionFaces, faceCentroids, sampleGenerators, subLabels, ClusterMetric::EuclideanDist,
					  params.hybridClusterMaxIters);

		for (std::size_t i = 0; i < regionFaces.size(); ++i)
		{
			faceToPatch[static_cast<std::size_t>(regionFaces[i])] = nextPatchId + subLabels[i];
		}
		int localMax = 0;
		for (const int l : subLabels)
		{
			localMax = std::max(localMax, l);
		}
		nextPatchId += localMax + 1;
	}

	for (int f = 0; f < faceCount; ++f)
	{
		if (faceToPatch[static_cast<std::size_t>(f)] < 0)
		{
			faceToPatch[static_cast<std::size_t>(f)] = 0;
		}
	}

	{
		std::unordered_map<int, int> remap;
		int dense = 0;
		for (int& p : faceToPatch)
		{
			auto it = remap.find(p);
			if (it == remap.end())
			{
				remap[p] = dense++;
				p = dense - 1;
			}
			else
			{
				p = it->second;
			}
		}
	}

	if (params.hybridEnableRegionAdjust)
	{
		HybridAdjustStats adjustStats;
		if (!hybridApplyRegionAdjust(mesh, params, adj.fullAdj, faceNormals, faceToPatch, adjustStats, errMsg))
		{
			return false;
		}
		if (partitionStats)
		{
			partitionStats->triPatchCount = adjustStats.triPatchCount;
			partitionStats->quadPatchCount = adjustStats.quadPatchCount;
			partitionStats->pentPatchCount = adjustStats.pentPatchCount;
			partitionStats->hexPatchCount = adjustStats.hexPatchCount;
		}
	}

	std::vector<int> chart = faceToPatch;
	chartToPatches(chart, patches);
	if (patches.empty())
	{
		if (errMsg)
		{
			*errMsg = "hybrid partition produced no patches";
		}
		return false;
	}

	rebuildPatchAdjacency(adj.fullAdj, faceCount, patches);
	outJunctionCount = computeJunctionCount(patches);

	int finalMin = 0;
	int finalMax = 0;
	int finalSmall = 0;
	computePatchFaceStats(patches, 1, finalMin, finalMax, finalSmall);
	if (partitionStats)
	{
		partitionStats->minFacesPerPatch = finalMin;
		partitionStats->maxFacesPerPatch = finalMax;
		partitionStats->smallPatchCount = finalSmall;
	}

	return true;
}

} // namespace meshrecon
} // namespace geoalgo
