#include "OtLcSkeleton.h"

#include <KdTreePointSet.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace geoalgo
{
namespace tg
{

namespace
{

inline double lengthSquared(const Vec3& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

double computeBboxDiagonal(const std::vector<float>& xyz)
{
	if (xyz.size() < 3U)
	{
		return 1.0;
	}
	Vec3 mn{
		std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max()};
	Vec3 mx{
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max()};
	for (std::size_t i = 0; i + 2U < xyz.size(); i += 3U)
	{
		const double x = static_cast<double>(xyz[i]);
		const double y = static_cast<double>(xyz[i + 1U]);
		const double z = static_cast<double>(xyz[i + 2U]);
		mn.x = std::min(mn.x, x);
		mn.y = std::min(mn.y, y);
		mn.z = std::min(mn.z, z);
		mx.x = std::max(mx.x, x);
		mx.y = std::max(mx.y, y);
		mx.z = std::max(mx.z, z);
	}
	return length(sub(mx, mn));
}

Vec3 computePrincipalAxisFromPointSet(const std::vector<Vec3>& points)
{
	if (points.empty())
	{
		return {1.0, 0.0, 0.0};
	}
	Vec3 mean{0.0, 0.0, 0.0};
	for (const Vec3& p : points)
	{
		mean = add(mean, p);
	}
	mean = scale(mean, 1.0 / static_cast<double>(points.size()));

	double cov[3][3] = {};
	for (const Vec3& p : points)
	{
		const Vec3 d = sub(p, mean);
		cov[0][0] += d.x * d.x;
		cov[0][1] += d.x * d.y;
		cov[0][2] += d.x * d.z;
		cov[1][1] += d.y * d.y;
		cov[1][2] += d.y * d.z;
		cov[2][2] += d.z * d.z;
	}
	cov[1][0] = cov[0][1];
	cov[2][0] = cov[0][2];
	cov[2][1] = cov[1][2];

	Vec3 axis{1.0, 0.0, 0.0};
	for (int iter = 0; iter < 24; ++iter)
	{
		const Vec3 next{
			cov[0][0] * axis.x + cov[0][1] * axis.y + cov[0][2] * axis.z,
			cov[1][0] * axis.x + cov[1][1] * axis.y + cov[1][2] * axis.z,
			cov[2][0] * axis.x + cov[2][1] * axis.y + cov[2][2] * axis.z};
		axis = normalizeVec3(next);
	}
	if (length(axis) < 1e-9)
	{
		axis = {1.0, 0.0, 0.0};
	}
	return axis;
}

bool computeTubePcaFrame(
	const std::vector<Vec3>& points,
	Vec3& outCentroid,
	Vec3& outAxis,
	double& outExtentMin,
	double& outExtentMax)
{
	outCentroid = {0.0, 0.0, 0.0};
	outAxis = {1.0, 0.0, 0.0};
	outExtentMin = 0.0;
	outExtentMax = 0.0;
	if (points.size() < 3U)
	{
		return false;
	}

	for (const Vec3& p : points)
	{
		outCentroid = add(outCentroid, p);
	}
	outCentroid = scale(outCentroid, 1.0 / static_cast<double>(points.size()));
	outAxis = computePrincipalAxisFromPointSet(points);

	double tMin = std::numeric_limits<double>::max();
	double tMax = -std::numeric_limits<double>::max();
	for (const Vec3& p : points)
	{
		const double t = dot(sub(p, outCentroid), outAxis);
		tMin = std::min(tMin, t);
		tMax = std::max(tMax, t);
	}
	outExtentMin = tMin;
	outExtentMax = tMax;
	return tMax > tMin + 1e-9;
}

bool extractSliceCentroidPolyline(
	const std::vector<Vec3>& points,
	const double binWidthMm,
	std::vector<Vec3>& outPolyline)
{
	outPolyline.clear();
	if (points.size() < 3U || binWidthMm <= 0.0)
	{
		return false;
	}

	Vec3 mean{0.0, 0.0, 0.0};
	Vec3 axis{1.0, 0.0, 0.0};
	double tMin = 0.0;
	double tMax = 0.0;
	if (!computeTubePcaFrame(points, mean, axis, tMin, tMax))
	{
		return false;
	}

	std::vector<std::pair<double, Vec3>> projected;
	projected.reserve(points.size());
	for (const Vec3& p : points)
	{
		projected.emplace_back(dot(sub(p, mean), axis), p);
	}
	std::sort(projected.begin(), projected.end(), [](const auto& a, const auto& b)
	{
		return a.first < b.first;
	});

	if (tMax - tMin < binWidthMm * 0.5)
	{
		return false;
	}

	for (double binStart = tMin; binStart <= tMax + 1e-9; binStart += binWidthMm)
	{
		const double binEnd = binStart + binWidthMm;
		Vec3 sum{0.0, 0.0, 0.0};
		int count = 0;
		for (const auto& entry : projected)
		{
			if (entry.first < binStart || entry.first >= binEnd)
			{
				continue;
			}
			sum = add(sum, entry.second);
			++count;
		}
		if (count <= 0)
		{
			continue;
		}
		outPolyline.push_back(scale(sum, 1.0 / static_cast<double>(count)));
	}

	return outPolyline.size() >= 2U;
}

std::vector<Vec3> subsamplePointsUniformLocal(
	const std::vector<Vec3>& points,
	const std::size_t maxCount)
{
	if (points.size() <= maxCount || maxCount < 2U)
	{
		return points;
	}
	std::vector<Vec3> out;
	out.reserve(maxCount);
	const double step = static_cast<double>(points.size() - 1U)
		/ static_cast<double>(maxCount - 1U);
	for (std::size_t i = 0; i < maxCount; ++i)
	{
		const std::size_t idx = static_cast<std::size_t>(step * static_cast<double>(i) + 0.5);
		out.push_back(points[std::min(idx, points.size() - 1U)]);
	}
	return out;
}

bool isCenterlinePolylineReasonable(
	const std::vector<Vec3>& polyline,
	const double maxArcToChordRatio)
{
	if (polyline.size() < 3U)
	{
		return true;
	}
	double arcLen = 0.0;
	for (std::size_t i = 1; i < polyline.size(); ++i)
	{
		arcLen += length(sub(polyline[i], polyline[i - 1U]));
	}
	const double chordLen = length(sub(polyline.back(), polyline.front()));
	if (chordLen < 1e-6)
	{
		return arcLen < 1e-3;
	}
	return arcLen / chordLen <= maxArcToChordRatio;
}

bool isSampleGraphUsable(
	const int nodeCount,
	const int edgeCount,
	const int componentCount)
{
	if (nodeCount < 2)
	{
		return false;
	}
	if (componentCount != 1)
	{
		return false;
	}
	return edgeCount >= nodeCount - 1;
}

bool extractClusterOrderedPolyline(
	const std::vector<Vec3>& rootPositions,
	const double sectionSpacingMm,
	std::vector<Vec3>& outPolyline)
{
	outPolyline.clear();
	if (rootPositions.size() < 2U)
	{
		return false;
	}

	Vec3 mean{0.0, 0.0, 0.0};
	Vec3 axis{1.0, 0.0, 0.0};
	double tMin = 0.0;
	double tMax = 0.0;
	if (!computeTubePcaFrame(rootPositions, mean, axis, tMin, tMax))
	{
		return false;
	}
	axis = normalizeVec3(axis);

	std::vector<std::pair<double, Vec3>> ordered;
	ordered.reserve(rootPositions.size());
	for (const Vec3& p : rootPositions)
	{
		ordered.emplace_back(dot(sub(p, mean), axis), p);
	}
	std::sort(ordered.begin(), ordered.end(), [](const auto& lhs, const auto& rhs)
	{
		return lhs.first < rhs.first;
	});

	const double mergeProj = sectionSpacingMm > 0.0 ? sectionSpacingMm * 0.45 : 1.0;
	for (const auto& entry : ordered)
	{
		if (outPolyline.empty())
		{
			outPolyline.push_back(entry.second);
			continue;
		}
		const double lastT = dot(sub(outPolyline.back(), mean), axis);
		if (entry.first - lastT >= mergeProj * 0.5)
		{
			outPolyline.push_back(entry.second);
		}
		else
		{
			outPolyline.back() = scale(add(outPolyline.back(), entry.second), 0.5);
		}
	}
	return outPolyline.size() >= 2U;
}

	struct VoxelKey
	{
		int64_t i = 0;
		int64_t j = 0;
		int64_t k = 0;

		bool operator==(const VoxelKey& o) const
		{
			return i == o.i && j == o.j && k == o.k;
		}
	};

	struct VoxelKeyHash
	{
		std::size_t operator()(const VoxelKey& key) const
		{
			return static_cast<std::size_t>(key.i ^ (key.j << 16) ^ (key.k << 32));
		}
	};

	struct VoxelAccum
	{
		Vec3 sum{0.0, 0.0, 0.0};
		int count = 0;
	};

	std::vector<Vec3> voxelDownsamplePoints(
		const std::vector<Vec3>& points,
		const double voxelSize)
	{
		if (points.empty() || voxelSize <= 0.0)
		{
			return {};
		}

		Vec3 mn{std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max()};
		Vec3 mx{-std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max()};
		for (const Vec3& p : points)
		{
			mn.x = std::min(mn.x, p.x);
			mn.y = std::min(mn.y, p.y);
			mn.z = std::min(mn.z, p.z);
			mx.x = std::max(mx.x, p.x);
			mx.y = std::max(mx.y, p.y);
			mx.z = std::max(mx.z, p.z);
		}

		const double invCell = 1.0 / voxelSize;
		std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;

		for (const Vec3& p : points)
		{
			const VoxelKey key{
				static_cast<int64_t>(std::floor((p.x - mn.x) * invCell)),
				static_cast<int64_t>(std::floor((p.y - mn.y) * invCell)),
				static_cast<int64_t>(std::floor((p.z - mn.z) * invCell))};
			VoxelAccum& accum = voxels[key];
			accum.sum = add(accum.sum, p);
			accum.count++;
		}

		std::vector<Vec3> centroids;
		centroids.reserve(voxels.size());
		for (const auto& kv : voxels)
		{
			centroids.push_back(scale(kv.second.sum, 1.0 / kv.second.count));
		}
		return centroids;
	}

int sampleFindRoot(std::vector<int>& parent, const int x)
{
	if (parent[static_cast<std::size_t>(x)] != x)
	{
		parent[static_cast<std::size_t>(x)] = sampleFindRoot(parent, parent[static_cast<std::size_t>(x)]);
	}
	return parent[static_cast<std::size_t>(x)];
}

void sampleUnite(
	std::vector<int>& parent,
	std::vector<Vec3>& positions,
	std::vector<double>& masses,
	const int a,
	const int b)
{
	int ra = sampleFindRoot(parent, a);
	int rb = sampleFindRoot(parent, b);
	if (ra == rb)
	{
		return;
	}
	const double ma = masses[static_cast<std::size_t>(ra)];
	const double mb = masses[static_cast<std::size_t>(rb)];
	const double msum = ma + mb;
	if (msum > 1e-12)
	{
		const Vec3 pa = positions[static_cast<std::size_t>(ra)];
		const Vec3 pb = positions[static_cast<std::size_t>(rb)];
		positions[static_cast<std::size_t>(ra)] = scale(add(scale(pa, ma), scale(pb, mb)), 1.0 / msum);
	}
	masses[static_cast<std::size_t>(ra)] = msum;
	parent[static_cast<std::size_t>(rb)] = ra;
}

void resetSampleUnionFind(OtSkeletonState& state)
{
	const int n = static_cast<int>(state.samplePositions.size());
	state.sampleParent.resize(static_cast<std::size_t>(n));
	std::iota(state.sampleParent.begin(), state.sampleParent.end(), 0);
}

void rebuildSampleGraphEdges(
	OtSkeletonState& state,
	const double linkDistMm,
	const double sectionSpacingMm);

bool otcClusterMergeStep(
	OtSkeletonState& state,
	const double mergeDistMm,
	const double sectionSpacingMm);

int sampleRootCount(const std::vector<int>& parent)
{
	int count = 0;
	for (int i = 0; i < static_cast<int>(parent.size()); ++i)
	{
		if (parent[static_cast<std::size_t>(i)] == i)
		{
			++count;
		}
	}
	return count;
}

std::vector<float> positionsToFloatXyz(const std::vector<Vec3>& positions)
{
	std::vector<float> xyz;
	xyz.reserve(positions.size() * 3U);
	for (const Vec3& p : positions)
	{
		xyz.push_back(static_cast<float>(p.x));
		xyz.push_back(static_cast<float>(p.y));
		xyz.push_back(static_cast<float>(p.z));
	}
	return xyz;
}

void collectActiveSampleRoots(
	const OtSkeletonState& state,
	std::vector<Vec3>& outRoots,
	std::vector<int>& outRootIndices)
{
	outRoots.clear();
	outRootIndices.clear();
	const int n = static_cast<int>(state.samplePositions.size());
	for (int i = 0; i < n; ++i)
	{
		if (sampleFindRoot(const_cast<std::vector<int>&>(state.sampleParent), i) != i)
		{
			continue;
		}
		outRoots.push_back(state.samplePositions[static_cast<std::size_t>(i)]);
		outRootIndices.push_back(i);
	}
}

void rebuildSampleTree(OtSkeletonState& state)
{
	std::vector<Vec3> roots;
	std::vector<int> rootIndices;
	collectActiveSampleRoots(state, roots, rootIndices);
	if (roots.size() < 2U)
	{
		state.sampleTree.reset();
		return;
	}
	const std::vector<float> rootXyz = positionsToFloatXyz(roots);
	auto tree = std::make_unique<pclalgo::KdTreePointSet>(rootXyz);
	if (!tree->empty())
	{
		state.sampleTree = std::move(tree);
	}
	else
	{
		state.sampleTree.reset();
	}
}

void assignOriginalPointsToSamples(OtSkeletonState& state)
{
	const int nOrig = static_cast<int>(state.originalPositions.size());
	state.originalCluster.assign(static_cast<std::size_t>(nOrig), 0);
	if (nOrig == 0)
	{
		return;
	}

	std::vector<Vec3> roots;
	std::vector<int> rootIndices;
	collectActiveSampleRoots(state, roots, rootIndices);
	if (roots.empty())
	{
		return;
	}

	rebuildSampleTree(state);
	if (!state.sampleTree)
	{
		return;
	}

	for (int oi = 0; oi < nOrig; ++oi)
	{
		const Vec3& y = state.originalPositions[static_cast<std::size_t>(oi)];
		std::vector<std::size_t> idx;
		std::vector<double> distSq;
		state.sampleTree->findKNearest(y.x, y.y, y.z, 1U, idx, distSq);
		if (idx.empty())
		{
			state.originalCluster[static_cast<std::size_t>(oi)] = rootIndices.front();
			continue;
		}
		const int local = static_cast<int>(idx.front());
		state.originalCluster[static_cast<std::size_t>(oi)] =
			rootIndices[static_cast<std::size_t>(local)];
	}
}

double updateSamplePositionsFromClusters(
	OtSkeletonState& state,
	const double beta,
	const double sinkhornEps,
	const int maxIters,
	const double energyEps)
{
	const int nSamples = static_cast<int>(state.samplePositions.size());
	const int nOrig = static_cast<int>(state.originalPositions.size());
	if (nSamples == 0 || nOrig == 0)
	{
		return 0.0;
	}

	const double invEps = (sinkhornEps > 1e-12) ? (1.0 / sinkhornEps) : 100.0;
	double maxMove = 0.0;

	for (int iter = 0; iter < maxIters; ++iter)
	{
		std::vector<Vec3> nextPositions(static_cast<std::size_t>(nSamples), Vec3{0.0, 0.0, 0.0});
		std::vector<double> nextMass(static_cast<std::size_t>(nSamples), 0.0);

		for (int oi = 0; oi < nOrig; ++oi)
		{
			const int cluster = state.originalCluster[static_cast<std::size_t>(oi)];
			if (cluster < 0 || cluster >= nSamples)
			{
				continue;
			}
			const Vec3& y = state.originalPositions[static_cast<std::size_t>(oi)];
			const Vec3& x = state.samplePositions[static_cast<std::size_t>(cluster)];
			const double d2 = lengthSquared(sub(x, y));
			const double c = std::pow(std::max(d2, 1e-12), beta);
			const double w = std::exp(-c * invEps) * state.originalMass[static_cast<std::size_t>(oi)];
			nextPositions[static_cast<std::size_t>(cluster)] =
				add(nextPositions[static_cast<std::size_t>(cluster)], scale(y, w));
			nextMass[static_cast<std::size_t>(cluster)] += w;
		}

		maxMove = 0.0;
		for (int si = 0; si < nSamples; ++si)
		{
			if (nextMass[static_cast<std::size_t>(si)] <= 1e-12)
			{
				continue;
			}
			const Vec3 updated = scale(
				nextPositions[static_cast<std::size_t>(si)],
				1.0 / nextMass[static_cast<std::size_t>(si)]);
			maxMove = std::max(maxMove, length(sub(updated, state.samplePositions[static_cast<std::size_t>(si)])));
			state.samplePositions[static_cast<std::size_t>(si)] = updated;
		}

		if (maxMove < energyEps)
		{
			break;
		}
	}
	return maxMove;
}

void markSkeletonFromSampleGraph(OtSkeletonState& state)
{
	const int n = static_cast<int>(state.samplePositions.size());
	state.isSampleSkeleton.assign(static_cast<std::size_t>(n), 0U);
	for (int i = 0; i < n; ++i)
	{
		if (sampleFindRoot(state.sampleParent, i) != i)
		{
			continue;
		}
		if (state.sampleEdges[static_cast<std::size_t>(i)].size() <= 1U)
		{
			state.isSampleSkeleton[static_cast<std::size_t>(i)] = 1U;
		}
	}

	const int nOrig = static_cast<int>(state.originalPositions.size());
	for (int oi = 0; oi < nOrig; ++oi)
	{
		const int cluster = state.originalCluster[static_cast<std::size_t>(oi)];
		if (cluster >= 0 && cluster < n && state.isSampleSkeleton[static_cast<std::size_t>(cluster)])
		{
			state.isSkeletonPoint[static_cast<std::size_t>(oi)] = 1U;
			state.isFixedPoint[static_cast<std::size_t>(oi)] = 1U;
		}
	}
}

void markAllActiveSamplesAsSkeleton(OtSkeletonState& state)
{
	const int n = static_cast<int>(state.samplePositions.size());
	for (int i = 0; i < n; ++i)
	{
		if (sampleFindRoot(state.sampleParent, i) == i)
		{
			state.isSampleSkeleton[static_cast<std::size_t>(i)] = 1U;
		}
	}
	markSkeletonFromSampleGraph(state);
}

bool otcClusterMergeStep(
	OtSkeletonState& state,
	const double mergeDistMm,
	const double sectionSpacingMm)
{
	const int n = static_cast<int>(state.samplePositions.size());
	if (n < 2)
	{
		return false;
	}

	bool anyMerge = false;
	std::vector<std::pair<double, std::pair<int, int>>> candidates;
	candidates.reserve(static_cast<std::size_t>(n) * 4U);
	for (int i = 0; i < n; ++i)
	{
		const int ri = sampleFindRoot(state.sampleParent, i);
		if (ri != i)
		{
			continue;
		}
		for (int j = i + 1; j < n; ++j)
		{
			const int rj = sampleFindRoot(state.sampleParent, j);
			if (rj != j || ri == rj)
			{
				continue;
			}
			const double d = length(sub(
				state.samplePositions[static_cast<std::size_t>(ri)],
				state.samplePositions[static_cast<std::size_t>(rj)]));
			if (d < mergeDistMm)
			{
				candidates.emplace_back(d, std::make_pair(ri, rj));
			}
		}
	}
	std::sort(candidates.begin(), candidates.end());
	for (const auto& entry : candidates)
	{
		const int ra = sampleFindRoot(state.sampleParent, entry.second.first);
		const int rb = sampleFindRoot(state.sampleParent, entry.second.second);
		if (ra == rb)
		{
			continue;
		}
		sampleUnite(state.sampleParent, state.samplePositions, state.sampleMass, ra, rb);
		anyMerge = true;
	}

	for (int oi = 0; oi < static_cast<int>(state.originalCluster.size()); ++oi)
	{
		state.originalCluster[static_cast<std::size_t>(oi)] =
			sampleFindRoot(state.sampleParent, state.originalCluster[static_cast<std::size_t>(oi)]);
	}

	rebuildSampleGraphEdges(state, mergeDistMm * 1.25, sectionSpacingMm);
	for (int i = 0; i < n; ++i)
	{
		if (sampleFindRoot(state.sampleParent, i) != i)
		{
			continue;
		}
		if (state.sampleEdges[static_cast<std::size_t>(i)].size() <= 1U)
		{
			state.isSampleSkeleton[static_cast<std::size_t>(i)] = 1U;
		}
	}
	return anyMerge;
}

void contractPointCloudConstrainedLc(
	std::vector<Vec3>& positions,
	const std::vector<Vec3>& anchors,
	const std::vector<std::vector<int>>& adjacency,
	const std::vector<uint8_t>& fixedMask,
	const double anchorWeight)
{
	const int n = static_cast<int>(positions.size());
	for (int i = 0; i < n; ++i)
	{
		if (i < static_cast<int>(fixedMask.size()) && fixedMask[static_cast<std::size_t>(i)])
		{
			continue;
		}
		const auto& neighbors = adjacency[static_cast<std::size_t>(i)];
		if (neighbors.empty())
		{
			continue;
		}
		Vec3 neighborSum{0.0, 0.0, 0.0};
		for (const int j : neighbors)
		{
			neighborSum = add(neighborSum, positions[static_cast<std::size_t>(j)]);
		}
		const double degree = static_cast<double>(neighbors.size());
		if (anchorWeight <= 0.0 || i >= static_cast<int>(anchors.size()))
		{
			positions[static_cast<std::size_t>(i)] = scale(neighborSum, 1.0 / degree);
		}
		else
		{
			const Vec3 numerator = add(
				neighborSum,
				scale(anchors[static_cast<std::size_t>(i)], anchorWeight));
			positions[static_cast<std::size_t>(i)] = scale(numerator, 1.0 / (degree + anchorWeight));
		}
	}
}

void filterMutualKnnAdjacency(std::vector<std::vector<int>>& adjacency)
{
	const int n = static_cast<int>(adjacency.size());
	for (int i = 0; i < n; ++i)
	{
		auto& neighbors = adjacency[static_cast<std::size_t>(i)];
		std::vector<int> kept;
		kept.reserve(neighbors.size());
		for (const int j : neighbors)
		{
			const auto& back = adjacency[static_cast<std::size_t>(j)];
			if (std::find(back.begin(), back.end(), i) != back.end())
			{
				kept.push_back(j);
			}
		}
		neighbors = std::move(kept);
	}
}

void estimatePointCloudInwardNormals(
	const std::vector<Vec3>& positions,
	const std::vector<std::vector<int>>& adjacency,
	std::vector<Vec3>& outNormals)
{
	const int n = static_cast<int>(positions.size());
	outNormals.assign(static_cast<std::size_t>(n), Vec3{0.0, 0.0, 1.0});
	if (n <= 0)
	{
		return;
	}

	Vec3 globalCentroid{0.0, 0.0, 0.0};
	for (const Vec3& p : positions)
	{
		globalCentroid = add(globalCentroid, p);
	}
	globalCentroid = scale(globalCentroid, 1.0 / static_cast<double>(n));

	for (int i = 0; i < n; ++i)
	{
		const auto& neighbors = adjacency[static_cast<std::size_t>(i)];
		if (neighbors.size() < 3U)
		{
			continue;
		}

		Vec3 localMean{0.0, 0.0, 0.0};
		for (const int j : neighbors)
		{
			localMean = add(localMean, positions[static_cast<std::size_t>(j)]);
		}
		localMean = scale(localMean, 1.0 / static_cast<double>(neighbors.size()));

		double cov[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
		for (const int j : neighbors)
		{
			const Vec3 d = sub(positions[static_cast<std::size_t>(j)], localMean);
			cov[0][0] += d.x * d.x;
			cov[0][1] += d.x * d.y;
			cov[0][2] += d.x * d.z;
			cov[1][1] += d.y * d.y;
			cov[1][2] += d.y * d.z;
			cov[2][2] += d.z * d.z;
		}
		cov[1][0] = cov[0][1];
		cov[2][0] = cov[0][2];
		cov[2][1] = cov[1][2];

		Vec3 normal;
		if (!smallestEigenvector3(cov, normal))
		{
			continue;
		}
		const Vec3 toCenter = sub(globalCentroid, positions[static_cast<std::size_t>(i)]);
		if (dot(normal, toCenter) < 0.0)
		{
			normal = scale(normal, -1.0);
		}
		outNormals[static_cast<std::size_t>(i)] = normalizeVec3(normal);
	}
}

void contractPointCloudInwardLc(
	std::vector<Vec3>& positions,
	const std::vector<Vec3>& anchors,
	const std::vector<std::vector<int>>& adjacency,
	const std::vector<Vec3>& inwardNormals,
	const std::vector<uint8_t>& fixedMask,
	const double anchorWeight,
	const double inwardStepMm,
	const int innerIters)
{
	const int n = static_cast<int>(positions.size());
	if (n <= 0 || innerIters <= 0)
	{
		return;
	}

	for (int pass = 0; pass < innerIters; ++pass)
	{
		contractPointCloudConstrainedLc(
			positions,
			anchors,
			adjacency,
			fixedMask,
			anchorWeight);

		if (inwardStepMm <= 1e-9)
		{
			continue;
		}

		for (int i = 0; i < n; ++i)
		{
			if (i < static_cast<int>(fixedMask.size()) && fixedMask[static_cast<std::size_t>(i)])
			{
				continue;
			}
			if (i >= static_cast<int>(inwardNormals.size()))
			{
				continue;
			}
			positions[static_cast<std::size_t>(i)] = add(
				positions[static_cast<std::size_t>(i)],
				scale(inwardNormals[static_cast<std::size_t>(i)], inwardStepMm));
		}
	}
}

void clampPositionsInwardOfInitialShell(
	std::vector<Vec3>& positions,
	const std::vector<Vec3>& initialShell,
	const std::vector<Vec3>& inwardNormals,
	const double maxInwardMm)
{
	const int n = static_cast<int>(positions.size());
	if (n <= 0
		|| static_cast<int>(initialShell.size()) != n
		|| static_cast<int>(inwardNormals.size()) != n)
	{
		return;
	}

	for (int i = 0; i < n; ++i)
	{
		const Vec3 nrm = inwardNormals[static_cast<std::size_t>(i)];
		if (length(nrm) < 1e-9)
		{
			continue;
		}
		const Vec3 unitN = normalizeVec3(nrm);
		const double inward = dot(sub(positions[static_cast<std::size_t>(i)], initialShell[static_cast<std::size_t>(i)]), unitN);
		if (inward < 0.0)
		{
			positions[static_cast<std::size_t>(i)] = initialShell[static_cast<std::size_t>(i)];
		}
		else if (maxInwardMm > 0.0 && inward > maxInwardMm)
		{
			positions[static_cast<std::size_t>(i)] = add(
				initialShell[static_cast<std::size_t>(i)],
				scale(unitN, maxInwardMm));
		}
	}
}

double estimateShellMaxInwardDistanceMm(
	const std::vector<Vec3>& shell,
	const std::vector<std::vector<int>>& adjacency,
	const double bboxDiag)
{
	const int n = static_cast<int>(shell.size());
	double thickness = bboxDiag * 0.04;
	for (int i = 0; i < n; ++i)
	{
		const auto& neighbors = adjacency[static_cast<std::size_t>(i)];
		for (const int j : neighbors)
		{
			thickness = std::max(thickness, length(sub(shell[static_cast<std::size_t>(j)], shell[static_cast<std::size_t>(i)])));
		}
	}
	return std::min(bboxDiag * 0.12, thickness * 0.55);
}

void refreshSampleMedialPositions(
	OtSkeletonState& state,
	const std::vector<Vec3>& inwardNormals,
	const double maxMeanDistanceMm)
{
	const int nOrig = static_cast<int>(state.originalPositions.size());
	const int nSamples = static_cast<int>(state.samplePositions.size());
	if (nOrig <= 0 || nSamples <= 0)
	{
		return;
	}

	if (state.originalCluster.size() != static_cast<std::size_t>(nOrig))
	{
		assignOriginalPointsToSamples(state);
	}
	if (state.originalCluster.size() != static_cast<std::size_t>(nOrig))
	{
		return;
	}

	std::unordered_map<int, std::vector<int>> rootToMembers;
	rootToMembers.reserve(static_cast<std::size_t>(nSamples) * 2U);
	for (int oi = 0; oi < nOrig; ++oi)
	{
		const int cluster = state.originalCluster[static_cast<std::size_t>(oi)];
		if (cluster < 0 || cluster >= nSamples)
		{
			continue;
		}
		const int root = sampleFindRoot(state.sampleParent, cluster);
		if (root < 0 || root >= nSamples)
		{
			continue;
		}
		rootToMembers[root].push_back(oi);
	}

	for (const auto& entry : rootToMembers)
	{
		const int root = entry.first;
		if (root < 0 || root >= nSamples)
		{
			continue;
		}
		const auto& members = entry.second;
		if (members.empty())
		{
			continue;
		}

		std::vector<Vec3> origins;
		std::vector<Vec3> dirs;
		origins.reserve(members.size());
		dirs.reserve(members.size());
		for (const int oi : members)
		{
			origins.push_back(state.originalPositions[static_cast<std::size_t>(oi)]);
			if (oi < static_cast<int>(inwardNormals.size()))
			{
				dirs.push_back(inwardNormals[static_cast<std::size_t>(oi)]);
			}
			else
			{
				dirs.push_back(Vec3{0.0, 0.0, 1.0});
			}
		}

		Vec3 medialCenter;
		if (members.size() >= 2U
			&& approximateRayBundleCenter(origins, dirs, maxMeanDistanceMm, medialCenter))
		{
			state.samplePositions[static_cast<std::size_t>(root)] = medialCenter;
		}
		else if (members.size() == 1U)
		{
			Vec3 pushDir = dirs.front();
			if (length(pushDir) < 1e-9)
			{
				pushDir = Vec3{0.0, 0.0, 1.0};
			}
			const double pushMm = maxMeanDistanceMm > 0.0 ? maxMeanDistanceMm * 0.35 : 1.0;
			state.samplePositions[static_cast<std::size_t>(root)] = add(
				origins.front(),
				scale(normalizeVec3(pushDir), pushMm));
		}
	}
}

void emitIterationSnapshot(
	OtSkeletonState& state,
	const std::vector<Vec3>& inwardNormals,
	const double medialTolMm,
	const int iteration,
	const OtLcIterationCallback& callback)
{
	if (!callback)
	{
		return;
	}
	refreshSampleMedialPositions(state, inwardNormals, medialTolMm);

	OtLcIterationSnapshot snap;
	snap.iteration = iteration;
	std::vector<int> rootIndices;
	collectActiveSampleRoots(state, snap.samplePositions, rootIndices);
	snap.contractedPositions = subsamplePointsUniformLocal(state.originalPositions, 8000U);
	callback(snap);
}

// 在根点集上将多个连通分量桥接为单一分量（跨分量最近根点对加边）。
// roots 为全局 sample 索引；state.sampleEdges 同样以全局索引寻址。
void bridgeSampleEdgeComponents(
	OtSkeletonState& state,
	const std::vector<int>& roots)
{
	const int rootCount = static_cast<int>(roots.size());
	if (rootCount < 2)
	{
		return;
	}

	std::unordered_map<int, int> rootToLocal;
	rootToLocal.reserve(static_cast<std::size_t>(rootCount) * 2U);
	for (int i = 0; i < rootCount; ++i)
	{
		rootToLocal[roots[static_cast<std::size_t>(i)]] = i;
	}

	// 本地连通分量标记（BFS over roots via sampleEdges）。
	std::vector<int> component(static_cast<std::size_t>(rootCount), -1);
	int componentCount = 0;
	for (int s = 0; s < rootCount; ++s)
	{
		if (component[static_cast<std::size_t>(s)] != -1)
		{
			continue;
		}
		const int label = componentCount++;
		std::vector<int> queue;
		queue.push_back(s);
		component[static_cast<std::size_t>(s)] = label;
		for (std::size_t head = 0; head < queue.size(); ++head)
		{
			const int localU = queue[head];
			const int globalU = roots[static_cast<std::size_t>(localU)];
			for (const int globalV : state.sampleEdges[static_cast<std::size_t>(globalU)])
			{
				const auto it = rootToLocal.find(globalV);
				if (it == rootToLocal.end())
				{
					continue;
				}
				const int localV = it->second;
				if (component[static_cast<std::size_t>(localV)] != -1)
				{
					continue;
				}
				component[static_cast<std::size_t>(localV)] = label;
				queue.push_back(localV);
			}
		}
	}

	// 反复合并最近的两个分量，直到只剩一个。
	while (componentCount > 1)
	{
		double bestDist = std::numeric_limits<double>::max();
		int bestLocalA = -1;
		int bestLocalB = -1;
		for (int a = 0; a < rootCount; ++a)
		{
			for (int b = a + 1; b < rootCount; ++b)
			{
				if (component[static_cast<std::size_t>(a)] == component[static_cast<std::size_t>(b)])
				{
					continue;
				}
				const double d = length(sub(
					state.samplePositions[static_cast<std::size_t>(roots[static_cast<std::size_t>(a)])],
					state.samplePositions[static_cast<std::size_t>(roots[static_cast<std::size_t>(b)])]));
				if (d < bestDist)
				{
					bestDist = d;
					bestLocalA = a;
					bestLocalB = b;
				}
			}
		}
		if (bestLocalA < 0 || bestLocalB < 0)
		{
			break;
		}

		const int globalA = roots[static_cast<std::size_t>(bestLocalA)];
		const int globalB = roots[static_cast<std::size_t>(bestLocalB)];
		state.sampleEdges[static_cast<std::size_t>(globalA)].push_back(globalB);
		state.sampleEdges[static_cast<std::size_t>(globalB)].push_back(globalA);

		// 将 B 所在分量标号并入 A 所在分量。
		const int labelKeep = component[static_cast<std::size_t>(bestLocalA)];
		const int labelDrop = component[static_cast<std::size_t>(bestLocalB)];
		for (int i = 0; i < rootCount; ++i)
		{
			if (component[static_cast<std::size_t>(i)] == labelDrop)
			{
				component[static_cast<std::size_t>(i)] = labelKeep;
			}
		}
		--componentCount;
	}

	for (const int globalIdx : roots)
	{
		auto& neighbors = state.sampleEdges[static_cast<std::size_t>(globalIdx)];
		std::sort(neighbors.begin(), neighbors.end());
		neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
	}
}

void rebuildSampleGraphEdges(
	OtSkeletonState& state,
	const double linkDistMm,
	const double sectionSpacingMm)
{
	const int n = static_cast<int>(state.samplePositions.size());
	state.sampleEdges.assign(static_cast<std::size_t>(n), {});
	if (n < 2)
	{
		return;
	}

	std::vector<int> roots;
	std::vector<Vec3> rootPositions;
	roots.reserve(static_cast<std::size_t>(n));
	rootPositions.reserve(static_cast<std::size_t>(n));
	for (int i = 0; i < n; ++i)
	{
		if (sampleFindRoot(state.sampleParent, i) == i)
		{
			roots.push_back(i);
			rootPositions.push_back(state.samplePositions[static_cast<std::size_t>(i)]);
		}
	}
	if (roots.size() < 2U)
	{
		return;
	}

	const double maxEdgeLen = std::max(
		linkDistMm,
		sectionSpacingMm > 0.0 ? sectionSpacingMm * 2.5 : linkDistMm);

	Vec3 axisMean{0.0, 0.0, 0.0};
	Vec3 tubeAxis{1.0, 0.0, 0.0};
	double tMin = 0.0;
	double tMax = 0.0;
	if (computeTubePcaFrame(rootPositions, axisMean, tubeAxis, tMin, tMax))
	{
		tubeAxis = normalizeVec3(tubeAxis);
	}

	const auto connectRoots = [&](const int ia, const int ib, const bool requireAxisAlign)
	{
		const Vec3 delta = sub(
			state.samplePositions[static_cast<std::size_t>(ib)],
			state.samplePositions[static_cast<std::size_t>(ia)]);
		const double dist = length(delta);
		if (dist <= 1e-9 || dist > maxEdgeLen)
		{
			return;
		}
		if (requireAxisAlign)
		{
			const double align = std::fabs(dot(normalizeVec3(delta), tubeAxis));
			if (align < 0.35)
			{
				return;
			}
		}
		state.sampleEdges[static_cast<std::size_t>(ia)].push_back(ib);
		state.sampleEdges[static_cast<std::size_t>(ib)].push_back(ia);
	};

	for (std::size_t a = 0; a < roots.size(); ++a)
	{
		for (std::size_t b = a + 1U; b < roots.size(); ++b)
		{
			connectRoots(roots[a], roots[b], true);
		}
	}

	if (computeTubePcaFrame(rootPositions, axisMean, tubeAxis, tMin, tMax))
	{
		std::vector<std::pair<double, int>> ordered;
		ordered.reserve(roots.size());
		for (std::size_t i = 0; i < roots.size(); ++i)
		{
			const double t = dot(
				sub(rootPositions[i], axisMean),
				normalizeVec3(tubeAxis));
			ordered.emplace_back(t, roots[i]);
		}
		std::sort(ordered.begin(), ordered.end(), [](const auto& lhs, const auto& rhs)
		{
			return lhs.first < rhs.first;
		});
		for (std::size_t i = 1; i < ordered.size(); ++i)
		{
			const int ia = ordered[i - 1U].second;
			const int ib = ordered[i].second;
			const Vec3 delta = sub(
				state.samplePositions[static_cast<std::size_t>(ib)],
				state.samplePositions[static_cast<std::size_t>(ia)]);
			if (length(delta) <= 1e-9)
			{
				continue;
			}
			state.sampleEdges[static_cast<std::size_t>(ia)].push_back(ib);
			state.sampleEdges[static_cast<std::size_t>(ib)].push_back(ia);
		}
	}

	const double knnLinkDist = std::max(
		maxEdgeLen * 3.0,
		sectionSpacingMm > 0.0 ? sectionSpacingMm * 12.0 : maxEdgeLen * 3.0);
	const std::vector<float> rootXyz = positionsToFloatXyz(rootPositions);
	pclalgo::KdTreePointSet rootTree(rootXyz);
	if (!rootTree.empty() && roots.size() >= 2U)
	{
		const int k = std::min(6, static_cast<int>(roots.size()) - 1);
		const unsigned int queryK = static_cast<unsigned int>(k) + 1U;
		for (std::size_t li = 0; li < roots.size(); ++li)
		{
			const Vec3& p = rootPositions[li];
			std::vector<std::size_t> idx;
			std::vector<double> distSq;
			rootTree.findKNearest(p.x, p.y, p.z, queryK, idx, distSq);
			for (std::size_t t = 0; t < idx.size(); ++t)
			{
				const std::size_t lj = idx[t];
				if (lj == li)
				{
					continue;
				}
				if (std::sqrt(distSq[t]) > knnLinkDist)
				{
					continue;
				}
				const int ia = roots[li];
				const int ib = roots[lj];
				state.sampleEdges[static_cast<std::size_t>(ia)].push_back(ib);
				state.sampleEdges[static_cast<std::size_t>(ib)].push_back(ia);
			}
		}
	}

	for (auto& neighbors : state.sampleEdges)
	{
		std::sort(neighbors.begin(), neighbors.end());
		neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
	}

	// 兜底：保证根点图连通。距离/方向阈值建边后可能残留多个连通分量
	// （根点过稀或弯管处 PCA 退化），这会让 extractCenterlineFromOtSkeleton
	// 的 isSampleGraphUsable 一票否决，进而掉到错误的全局 PCA 兜底。
	// 这里在根点上反复用「跨分量最近根点对」加桥边，直到单一连通分量，
	// 等价于在根点上补一棵跨分量最小生成树。
	bridgeSampleEdgeComponents(state, roots);
}

bool sparseMergeSampleRoots(
	OtSkeletonState& state,
	const int targetRootCount,
	const double mergeDistStartMm,
	const double mergeDistMaxMm,
	const double sectionSpacingMm)
{
	if (targetRootCount <= 0 || mergeDistStartMm <= 0.0)
	{
		return false;
	}

	bool anyMerge = false;
	double mergeDist = mergeDistStartMm;
	while (sampleRootCount(state.sampleParent) > targetRootCount && mergeDist <= mergeDistMaxMm)
	{
		if (otcClusterMergeStep(state, mergeDist, sectionSpacingMm))
		{
			anyMerge = true;
		}
		mergeDist *= 1.25;
	}
	return anyMerge;
}

void refineRootPositionsInward(
	const OtSkeletonState& state,
	const std::vector<Vec3>& inwardNormals,
	const std::vector<int>& rootIndices,
	const double maxMeanDistanceMm,
	std::vector<Vec3>& inOutRootPositions)
{
	if (inOutRootPositions.size() != rootIndices.size())
	{
		return;
	}

	std::unordered_map<int, std::vector<int>> rootToMembers;
	rootToMembers.reserve(rootIndices.size() * 2U);
	const int nOrig = static_cast<int>(state.originalPositions.size());
	const int nSamples = static_cast<int>(state.samplePositions.size());
	for (int oi = 0; oi < nOrig; ++oi)
	{
		const int cluster = state.originalCluster[static_cast<std::size_t>(oi)];
		if (cluster < 0 || cluster >= nSamples)
		{
			continue;
		}
		const int root = sampleFindRoot(const_cast<std::vector<int>&>(state.sampleParent), cluster);
		rootToMembers[root].push_back(oi);
	}

	for (std::size_t li = 0; li < rootIndices.size(); ++li)
	{
		const int root = rootIndices[li];
		const auto it = rootToMembers.find(root);
		if (it == rootToMembers.end() || it->second.size() < 3U)
		{
			continue;
		}

		std::vector<Vec3> origins;
		std::vector<Vec3> dirs;
		origins.reserve(it->second.size());
		dirs.reserve(it->second.size());
		for (const int oi : it->second)
		{
			origins.push_back(state.originalPositions[static_cast<std::size_t>(oi)]);
			if (oi < static_cast<int>(inwardNormals.size()))
			{
				dirs.push_back(inwardNormals[static_cast<std::size_t>(oi)]);
			}
			else
			{
				dirs.push_back(Vec3{0.0, 0.0, 1.0});
			}
		}

		Vec3 medialCenter;
		if (approximateRayBundleCenter(origins, dirs, maxMeanDistanceMm, medialCenter))
		{
			inOutRootPositions[li] = medialCenter;
		}
	}
}

bool buildWeldedMeshPositions(
	const IndexedMeshLite& mesh,
	std::vector<Vec3>& outPositions,
	std::vector<std::vector<int>>& outAdjacency)
{
	const int vertexCount = countWeldedVertices(mesh);
	if (vertexCount <= 0)
	{
		return false;
	}
	outPositions.assign(static_cast<std::size_t>(vertexCount), Vec3{});
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		const auto& face = mesh.faceVerts[static_cast<std::size_t>(f)];
		outPositions[static_cast<std::size_t>(face[0])] = {
			mesh.soup[base + 0], mesh.soup[base + 1], mesh.soup[base + 2]};
		outPositions[static_cast<std::size_t>(face[1])] = {
			mesh.soup[base + 3], mesh.soup[base + 4], mesh.soup[base + 5]};
		outPositions[static_cast<std::size_t>(face[2])] = {
			mesh.soup[base + 6], mesh.soup[base + 7], mesh.soup[base + 8]};
	}
	outAdjacency = buildVertexAdjacency(mesh);
	return !outPositions.empty();
}

bool buildOriginalFromInput(
	const SkeletonInput& input,
	std::vector<Vec3>& outOriginal,
	std::vector<std::vector<int>>* outAdjacency,
	std::string* errMsg)
{
	outOriginal.clear();
	if (outAdjacency)
	{
		outAdjacency->clear();
	}

	if (input.kind == SkeletonInputKind::Mesh && input.mesh)
	{
		std::vector<std::vector<int>> meshAdjacency;
		std::vector<std::vector<int>>& adjTarget = outAdjacency ? *outAdjacency : meshAdjacency;
		if (!buildWeldedMeshPositions(*input.mesh, outOriginal, adjTarget))
		{
			if (errMsg)
			{
				*errMsg = "failed to build welded mesh positions";
			}
			return false;
		}
	}
	else if (input.pointXyz)
	{
		const auto& xyz = *input.pointXyz;
		outOriginal.reserve(xyz.size() / 3U);
		for (std::size_t i = 0; i + 2U < xyz.size(); i += 3U)
		{
			outOriginal.push_back({
				static_cast<double>(xyz[i]),
				static_cast<double>(xyz[i + 1U]),
				static_cast<double>(xyz[i + 2U])});
		}
	}
	else
	{
		if (errMsg)
		{
			*errMsg = "invalid skeleton input";
		}
		return false;
	}

	if (outOriginal.size() < 3U)
	{
		if (errMsg)
		{
			*errMsg = "too few input points";
		}
		return false;
	}
	return true;
}

void runOtUpdateAndOtc(
	OtSkeletonState& state,
	const OtLcParams& params,
	const double mergeDistMm,
	const double sectionSpacingMm,
	const std::vector<Vec3>& inwardNormals,
	const double medialTolMm,
	bool& outAnyMerge)
{
	assignOriginalPointsToSamples(state);
	updateSamplePositionsFromClusters(
		state,
		params.otCostBeta,
		params.otSinkhornEps,
		params.otSinkhornIters,
		params.otLcEnergyEps);
	refreshSampleMedialPositions(state, inwardNormals, medialTolMm);
	outAnyMerge = otcClusterMergeStep(state, mergeDistMm, sectionSpacingMm);
	refreshSampleMedialPositions(state, inwardNormals, medialTolMm);
}

void buildRootAdjacencyFromSampleEdges(
	const OtSkeletonState& state,
	const std::vector<int>& rootIndices,
	std::vector<std::vector<int>>& outAdjacency)
{
	const int n = static_cast<int>(rootIndices.size());
	outAdjacency.assign(static_cast<std::size_t>(n), {});
	std::unordered_map<int, int> rootToLocal;
	rootToLocal.reserve(static_cast<std::size_t>(n) * 2U);
	for (int i = 0; i < n; ++i)
	{
		rootToLocal[rootIndices[static_cast<std::size_t>(i)]] = i;
	}

	for (int li = 0; li < n; ++li)
	{
		const int ri = rootIndices[static_cast<std::size_t>(li)];
		for (const int nb : state.sampleEdges[static_cast<std::size_t>(ri)])
		{
			const int rnb = sampleFindRoot(const_cast<std::vector<int>&>(state.sampleParent), nb);
			const auto it = rootToLocal.find(rnb);
			if (it == rootToLocal.end() || it->second == li)
			{
				continue;
			}
			outAdjacency[static_cast<std::size_t>(li)].push_back(it->second);
		}
	}
	for (auto& neighbors : outAdjacency)
	{
		std::sort(neighbors.begin(), neighbors.end());
		neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
	}
}

bool buildKnnAdjacencyFromTree(
	const pclalgo::KdTreePointSet& tree,
	const std::vector<float>& xyz,
	int k,
	std::vector<std::vector<int>>& outAdjacency,
	std::string* errMsg)
{
	outAdjacency.clear();
	const std::size_t n = xyz.size() / 3U;
	if (n < 2U || k <= 0 || tree.empty() || tree.size() != n)
	{
		if (errMsg)
		{
			*errMsg = "invalid point cloud for knn";
		}
		return false;
	}

	outAdjacency.assign(n, {});
	const int kk = std::min(k, static_cast<int>(n) - 1);
	const unsigned int queryK = static_cast<unsigned int>(kk) + 1U;

	for (std::size_t i = 0; i < n; ++i)
	{
		const double px = static_cast<double>(xyz[i * 3U]);
		const double py = static_cast<double>(xyz[i * 3U + 1U]);
		const double pz = static_cast<double>(xyz[i * 3U + 2U]);

		std::vector<std::size_t> neighborIdx;
		std::vector<double> neighborDistSq;
		tree.findKNearest(px, py, pz, queryK, neighborIdx, neighborDistSq);

		outAdjacency[i].reserve(static_cast<std::size_t>(kk));
		for (std::size_t t = 0; t < neighborIdx.size() && static_cast<int>(outAdjacency[i].size()) < kk; ++t)
		{
			const int j = static_cast<int>(neighborIdx[t]);
			if (j < 0 || static_cast<std::size_t>(j) == i)
			{
				continue;
			}
			outAdjacency[i].push_back(j);
		}
	}
	return true;
}

} // namespace

int countUndirectedEdgesInAdjacency(const std::vector<std::vector<int>>& adjacency)
{
	std::size_t sum = 0U;
	for (const auto& neighbors : adjacency)
	{
		sum += neighbors.size();
	}
	return static_cast<int>(sum / 2U);
}

int countConnectedComponentsInAdjacency(const std::vector<std::vector<int>>& adjacency)
{
	const int n = static_cast<int>(adjacency.size());
	if (n == 0)
	{
		return 0;
	}
	std::vector<int> visited(static_cast<std::size_t>(n), 0);
	int components = 0;
	for (int start = 0; start < n; ++start)
	{
		if (visited[static_cast<std::size_t>(start)])
		{
			continue;
		}
		++components;
		std::vector<int> queue;
		queue.push_back(start);
		visited[static_cast<std::size_t>(start)] = 1;
		for (std::size_t head = 0; head < queue.size(); ++head)
		{
			const int u = queue[head];
			for (const int v : adjacency[static_cast<std::size_t>(u)])
			{
				if (v < 0 || v >= n || visited[static_cast<std::size_t>(v)])
				{
					continue;
				}
				visited[static_cast<std::size_t>(v)] = 1;
				queue.push_back(v);
			}
		}
	}
	return components;
}

void fillOtLcGraphDiagnostics(
	const std::vector<Vec3>& samplePositions,
	const std::vector<std::vector<int>>& adjacency,
	const bool usedKnnFallback,
	OtLcGraphDiagnostics& outDiagnostics)
{
	outDiagnostics.rootSampleCount = static_cast<int>(samplePositions.size());
	outDiagnostics.undirectedEdgeCount = countUndirectedEdgesInAdjacency(adjacency);
	outDiagnostics.connectedComponentCount = countConnectedComponentsInAdjacency(adjacency);
	outDiagnostics.usedKnnFallbackEdges = usedKnnFallback;
}

bool buildPointCloudKnnDknnAdjacency(
	const std::vector<float>& xyz,
	int k,
	std::vector<std::vector<int>>& outAdjacency,
	std::string* errMsg)
{
	pclalgo::KdTreePointSet tree(xyz);
	if (tree.empty())
	{
		if (errMsg)
		{
			*errMsg = "failed to build point kd-tree";
		}
		return false;
	}
	if (!buildKnnAdjacencyFromTree(tree, xyz, k, outAdjacency, errMsg))
	{
		return false;
	}
	filterMutualKnnAdjacency(outAdjacency);
	return true;
}

OtLcParams buildOtLcParams(const TubularGrindingParams& params)
{
	OtLcParams p;
	p.centerlineIterations = params.centerlineIterations > 0 ? params.centerlineIterations : 80;
	p.laplacianLambda = params.laplacianLambda;
	p.laplacianAttraction = params.laplacianAttraction;
	p.sectionSpacingMm = params.sectionSpacingMm > 0.0 ? params.sectionSpacingMm : 2.0;
	p.otSampleRate = params.otSampleRate > 0.0 ? params.otSampleRate : 0.10;
	p.otCostBeta = params.otCostBeta > 0.0 ? params.otCostBeta : 3.0;
	p.otcPreSteps = params.otcPreSteps > 0 ? params.otcPreSteps : 3;
	p.otcOuterLoops = params.otcOuterLoops > 0 ? params.otcOuterLoops : 3;
	p.otLcOuterMaxIters = params.otLcOuterMaxIters > 0 ? params.otLcOuterMaxIters : 40;
	p.minRootsBySamples = params.minRootsBySamples >= 0 ? params.minRootsBySamples : 0;
	p.pointCloudKnnK = params.pointCloudKnnK > 0 ? params.pointCloudKnnK : 30;
	return p;
}

bool extractCenterlineFromOtSkeleton(
	const std::vector<Vec3>& samplePositions,
	const std::vector<std::vector<int>>& sampleEdges,
	std::vector<Vec3>& outPolyline,
	const double sectionSpacingMm,
	OtLcGraphDiagnostics* outDiagnostics)
{
	(void)sectionSpacingMm;
	outPolyline.clear();
	if (samplePositions.size() < 2U)
	{
		return false;
	}

	std::vector<std::vector<int>> adjacency = sampleEdges;
	if (adjacency.size() != samplePositions.size())
	{
		adjacency.assign(samplePositions.size(), {});
	}

	bool hasEdge = false;
	for (const auto& neighbors : adjacency)
	{
		if (!neighbors.empty())
		{
			hasEdge = true;
			break;
		}
	}
	if (!hasEdge)
	{
		return false;
	}

	const int nodeCount = static_cast<int>(samplePositions.size());
	const int edgeCount = countUndirectedEdgesInAdjacency(adjacency);
	const int componentCount = countConnectedComponentsInAdjacency(adjacency);
	if (!isSampleGraphUsable(nodeCount, edgeCount, componentCount))
	{
		return false;
	}

	if (outDiagnostics)
	{
		fillOtLcGraphDiagnostics(samplePositions, adjacency, false, *outDiagnostics);
	}

	return extractLongestPathPolylineFromGraph(samplePositions, adjacency, outPolyline);
}

bool runOtLcSkeletonCenterline(
	const SkeletonInput& input,
	const TubularGrindingParams& params,
	std::vector<TubularCenterlineSample>& outSamples,
	TubularCenterlinePcaAxis* outPcaAxis,
	std::string* errMsg,
	bool* outCenterlinePcaFallback,
	OtLcGraphDiagnostics* outGraphDiagnostics,
	OtLcIterationCallback onIteration)
{
	outSamples.clear();
	if (outCenterlinePcaFallback)
	{
		*outCenterlinePcaFallback = false;
	}
	if (outGraphDiagnostics)
	{
		*outGraphDiagnostics = OtLcGraphDiagnostics{};
	}
	if (outPcaAxis)
	{
		*outPcaAxis = TubularCenterlinePcaAxis{};
	}

	const OtLcParams otParams = buildOtLcParams(params);

	std::vector<std::vector<int>> lcAdjacency;
	std::vector<Vec3> original;
	if (!buildOriginalFromInput(input, original, &lcAdjacency, errMsg))
	{
		return false;
	}

	const std::vector<float> xyz = input.pointXyz && !input.pointXyz->empty()
		? *input.pointXyz
		: positionsToFloatXyz(original);
	if (xyz.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "insufficient points for otlc";
		}
		return false;
	}

	const double bboxDiag = computeBboxDiagonal(xyz);

	if (lcAdjacency.empty())
	{
		pclalgo::KdTreePointSet pointTree(xyz);
		if (pointTree.empty())
		{
			if (errMsg)
			{
				*errMsg = "failed to build point kd-tree";
			}
			return false;
		}
		if (!buildKnnAdjacencyFromTree(
				pointTree,
				xyz,
				otParams.pointCloudKnnK,
				lcAdjacency,
				errMsg))
		{
			return false;
		}
		filterMutualKnnAdjacency(lcAdjacency);
	}

	OtSkeletonState state;
	state.originalPositions = original;
	state.anchorPositions = original;
	state.originalMass.assign(
		state.originalPositions.size(),
		1.0 / static_cast<double>(state.originalPositions.size()));

	const double voxelSize = bboxDiag / std::pow(
		std::max(1.0, static_cast<double>(original.size()) * otParams.otSampleRate),
		1.0 / 3.0);
	std::vector<Vec3> voxelCentroids = voxelDownsamplePoints(original, voxelSize);
	if (voxelCentroids.size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "voxel downsampling failed";
		}
		return false;
	}

	const std::vector<float> origXyz = positionsToFloatXyz(original);
	pclalgo::KdTreePointSet origTree(origXyz);
	state.samplePositions.reserve(voxelCentroids.size());
	state.sampleToOriginal.reserve(voxelCentroids.size());
	for (const Vec3& c : voxelCentroids)
	{
		std::vector<std::size_t> idx;
		std::vector<double> distSq;
		origTree.findKNearest(c.x, c.y, c.z, 1U, idx, distSq);
		if (!idx.empty())
		{
			const int nearest = static_cast<int>(idx.front());
			state.samplePositions.push_back(original[static_cast<std::size_t>(nearest)]);
			state.sampleToOriginal.push_back(nearest);
		}
	}
	if (state.samplePositions.size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "snapped downsampling failed";
		}
		return false;
	}
	state.sampleMass.assign(
		state.samplePositions.size(),
		1.0 / static_cast<double>(state.samplePositions.size()));
	state.isSkeletonPoint.assign(state.originalPositions.size(), 0U);
	state.isFixedPoint.assign(state.originalPositions.size(), 0U);
	state.isSampleSkeleton.assign(state.samplePositions.size(), 0U);
	resetSampleUnionFind(state);
	rebuildSampleTree(state);

	const double weightStart = std::max(1.0, otParams.laplacianAttraction * 5.0);
	const double weightPeak = std::clamp(otParams.laplacianLambda * 500.0, 10.0, 200.0);
	const double sectionSpacing = otParams.sectionSpacingMm;
	const double preMergeDist = std::max(bboxDiag * 0.025, sectionSpacing * 2.0);
	const double loopMergeDist = std::max(bboxDiag * 0.03, sectionSpacing * 2.5);
	const double medialTolMm = std::max(sectionSpacing * 1.5, bboxDiag * 0.01);
	const int lcInnerPerOuter = std::max(
		2,
		otParams.centerlineIterations / std::max(1, otParams.otLcOuterMaxIters));

	// 根点合并下限：仅以 bboxDiag/sectionSpacing 估计目标根数时，点云尺度偏小
	// 或 sectionSpacingMm 偏大会把目标根数压到几十个，导致骨架过度合并、
	// 图碎裂并掉到错误的全局 PCA 兜底。这里附加一个相对样本数的下限
	// （并 clamp 到样本总数，避免在小点云上设置达不到的下限而抑制合并）。
	const int sampleCount = static_cast<int>(state.samplePositions.size());
	const int minRootsBySamples = otParams.minRootsBySamples > 0
		? std::min(sampleCount, otParams.minRootsBySamples)
		: std::min(sampleCount, std::max(15, static_cast<int>(sampleCount * 0.05)));

	const std::vector<Vec3> initialShellPositions = state.originalPositions;
	std::vector<Vec3> shellInwardNormals;
	estimatePointCloudInwardNormals(initialShellPositions, lcAdjacency, shellInwardNormals);
	const double maxInwardMm = estimateShellMaxInwardDistanceMm(
		initialShellPositions,
		lcAdjacency,
		bboxDiag);
	const int minOuterItersBeforeStop = std::min(
		otParams.otLcOuterMaxIters,
		std::max(15, otParams.centerlineIterations / 5));

	for (int step = 0; step < otParams.otcPreSteps; ++step)
	{
		bool merged = false;
		runOtUpdateAndOtc(
			state,
			otParams,
			preMergeDist,
			sectionSpacing,
			shellInwardNormals,
			medialTolMm,
			merged);
		(void)merged;
	}

	emitIterationSnapshot(state, shellInwardNormals, medialTolMm, 0, onIteration);

	for (int outer = 0; outer < otParams.otLcOuterMaxIters; ++outer)
	{
		const double anchorWeight = computeContractionAnchorWeight(
			outer,
			std::max(1, otParams.otLcOuterMaxIters),
			weightStart,
			weightPeak);
		if (anchorWeight <= weightPeak * 0.25)
		{
			state.anchorPositions = state.originalPositions;
		}

		const double anchorRatio = std::min(1.0, anchorWeight / std::max(weightPeak, 1e-9));
		const double inwardBase = std::max(
			bboxDiag * 0.0025,
			sectionSpacing * 0.12);
		const double inwardStep = inwardBase * (0.15 + 0.85 * (1.0 - anchorRatio));
		contractPointCloudInwardLc(
			state.originalPositions,
			state.anchorPositions,
			lcAdjacency,
			shellInwardNormals,
			state.isFixedPoint,
			anchorWeight,
			inwardStep,
			lcInnerPerOuter);
		clampPositionsInwardOfInitialShell(
			state.originalPositions,
			initialShellPositions,
			shellInwardNormals,
			maxInwardMm);
		refreshSampleMedialPositions(state, shellInwardNormals, medialTolMm);

		bool anyMerge = false;
		for (int k = 0; k < otParams.otcOuterLoops; ++k)
		{
			bool merged = false;
			runOtUpdateAndOtc(
				state,
				otParams,
				loopMergeDist,
				sectionSpacing,
				shellInwardNormals,
				medialTolMm,
				merged);
			anyMerge = anyMerge || merged;
		}

		const int targetRoots = std::max(
			std::max(8, minRootsBySamples),
			static_cast<int>(bboxDiag / std::max(sectionSpacing, 1.0) * 0.6));
		if (sampleRootCount(state.sampleParent) > targetRoots)
		{
			anyMerge = sparseMergeSampleRoots(
				state,
				targetRoots,
				loopMergeDist,
				bboxDiag * 0.12,
				sectionSpacing) || anyMerge;
			if (anyMerge)
			{
				rebuildSampleTree(state);
			}
		}

		emitIterationSnapshot(state, shellInwardNormals, medialTolMm, outer + 1, onIteration);

		if (outer + 1 >= minOuterItersBeforeStop)
		{
			const int rootCount = sampleRootCount(state.sampleParent);
			if (rootCount <= targetRoots && !anyMerge)
			{
				break;
			}
		}
	}

	const int finalTargetRoots = std::max(
		std::max(6, minRootsBySamples),
		static_cast<int>(bboxDiag / std::max(sectionSpacing, 1.0) * 0.45));
	sparseMergeSampleRoots(
		state,
		finalTargetRoots,
		loopMergeDist,
		bboxDiag * 0.15,
		sectionSpacing);
	rebuildSampleGraphEdges(state, loopMergeDist * 1.25, sectionSpacing);
	rebuildSampleTree(state);

	std::vector<Vec3> rootPositions;
	std::vector<int> rootIndices;
	collectActiveSampleRoots(state, rootPositions, rootIndices);
	if (rootPositions.size() < 2U)
	{
		rootPositions = state.samplePositions;
		rootIndices.clear();
		rootIndices.reserve(rootPositions.size());
		for (int i = 0; i < static_cast<int>(rootPositions.size()); ++i)
		{
			rootIndices.push_back(i);
		}
	}

	refineRootPositionsInward(state, shellInwardNormals, rootIndices, medialTolMm, rootPositions);

	std::vector<std::vector<int>> rootAdjacency;
	buildRootAdjacencyFromSampleEdges(state, rootIndices, rootAdjacency);

	const std::vector<Vec3> contractedCloud = subsamplePointsUniformLocal(
		state.originalPositions,
		12000U);

	std::vector<Vec3> polyline;
	OtLcGraphDiagnostics graphDiag;
	graphDiag.rootSampleCount = static_cast<int>(rootPositions.size());
	graphDiag.undirectedEdgeCount = countUndirectedEdgesInAdjacency(rootAdjacency);
	graphDiag.connectedComponentCount = countConnectedComponentsInAdjacency(rootAdjacency);

	constexpr double kMaxArcToChord = 4.5;
	int extractPathKind = 2;
	if (extractClusterOrderedPolyline(rootPositions, sectionSpacing, polyline)
		&& isCenterlinePolylineReasonable(polyline, kMaxArcToChord))
	{
		extractPathKind = 1;
	}
	else if (extractCenterlineFromOtSkeleton(
			rootPositions,
			rootAdjacency,
			polyline,
			sectionSpacing,
			&graphDiag)
		&& isCenterlinePolylineReasonable(polyline, kMaxArcToChord))
	{
		extractPathKind = 1;
	}
	else
	{
		polyline.clear();
		if (extractSliceCentroidPolyline(contractedCloud, sectionSpacing, polyline)
			&& isCenterlinePolylineReasonable(polyline, kMaxArcToChord))
		{
			extractPathKind = 0;
		}
		else
		{
			polyline.clear();
			if (!extractOrderedCenterlinePolyline(contractedCloud, sectionSpacing, polyline))
			{
				if (errMsg)
				{
					*errMsg = "otlc centerline polyline extraction failed";
				}
				return false;
			}
			extractPathKind = 2;
		}
	}

	graphDiag.extractPathKind = extractPathKind;
	if (outGraphDiagnostics)
	{
		*outGraphDiagnostics = graphDiag;
	}
	if (outCenterlinePcaFallback)
	{
		*outCenterlinePcaFallback = extractPathKind != 1;
	}

	resamplePolylineToSamples(polyline, otParams.sectionSpacingMm, outSamples);
	if (outSamples.size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "resample failed";
		}
		return false;
	}

	std::vector<TubularCenterlineSample> framed;
	buildFrenetFrames(outSamples, framed);
	outSamples = std::move(framed);

	if (outPcaAxis && polyline.size() >= 3U)
	{
		computeCenterlinePcaAxisFromPoints(polyline, *outPcaAxis);
	}
	return true;
}

} // namespace tg
} // namespace geoalgo
