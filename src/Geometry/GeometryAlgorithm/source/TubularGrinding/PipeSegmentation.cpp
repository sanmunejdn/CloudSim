#include "PipeSegmentation.h"

#include "TubularGrindingCommon.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{

namespace
{

constexpr int kFaceUnassigned = -1;
constexpr int kFaceJunction = -2;
constexpr double kPlanarNormalSpreadDeg = 3.0;

struct UnionFind
{
	std::vector<int> parent;
	explicit UnionFind(const int n) : parent(static_cast<std::size_t>(n))
	{
		for (int i = 0; i < n; ++i)
		{
			parent[static_cast<std::size_t>(i)] = i;
		}
	}

	int find(int x)
	{
		while (parent[static_cast<std::size_t>(x)] != x)
		{
			parent[static_cast<std::size_t>(x)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(x)])];
			x = parent[static_cast<std::size_t>(x)];
		}
		return x;
	}

	void unite(const int a, const int b)
	{
		const int ra = find(a);
		const int rb = find(b);
		if (ra != rb)
		{
			parent[static_cast<std::size_t>(rb)] = ra;
		}
	}
};

double meshBBoxDiagonalMm(const IndexedMeshLite& mesh)
{
	const double dx = mesh.bboxMax[0] - mesh.bboxMin[0];
	const double dy = mesh.bboxMax[1] - mesh.bboxMin[1];
	const double dz = mesh.bboxMax[2] - mesh.bboxMin[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double estimateRingClusterEpsMm(const IndexedMeshLite& mesh, const std::vector<Vec3>& validCenters)
{
	if (validCenters.size() < 4U)
	{
		return std::max(5.0, meshBBoxDiagonalMm(mesh) * 0.01);
	}
	std::vector<double> nnDist;
	nnDist.reserve(validCenters.size());
	for (std::size_t i = 0; i < validCenters.size(); ++i)
	{
		double best = std::numeric_limits<double>::max();
		for (std::size_t j = 0; j < validCenters.size(); ++j)
		{
			if (i == j)
			{
				continue;
			}
			best = std::min(best, length(sub(validCenters[j], validCenters[i])));
		}
		if (best < std::numeric_limits<double>::max())
		{
			nnDist.push_back(best);
		}
	}
	if (nnDist.empty())
	{
		return std::max(5.0, meshBBoxDiagonalMm(mesh) * 0.01);
	}
	const std::size_t mid = nnDist.size() / 2U;
	std::nth_element(nnDist.begin(), nnDist.begin() + static_cast<std::ptrdiff_t>(mid), nnDist.end());
	return std::clamp(nnDist[mid] * 0.72, 4.0, meshBBoxDiagonalMm(mesh) * 0.08);
}

void splitClusterByConnectivity(
	const IndexedMeshLite& mesh,
	const std::vector<int>& clusterFaces,
	std::vector<std::vector<int>>& outComponents)
{
	outComponents.clear();
	if (clusterFaces.empty())
	{
		return;
	}
	std::set<int> allowed(clusterFaces.begin(), clusterFaces.end());
	std::map<int, int> faceToLocal;
	int local = 0;
	for (const int f : clusterFaces)
	{
		faceToLocal[f] = local++;
	}
	std::vector<uint8_t> visited(clusterFaces.size(), 0U);
	for (const int seed : clusterFaces)
	{
		const std::size_t seedIdx = static_cast<std::size_t>(faceToLocal[seed]);
		if (visited[seedIdx] != 0U)
		{
			continue;
		}
		std::vector<int> comp;
		std::queue<int> q;
		q.push(seed);
		visited[seedIdx] = 1U;
		while (!q.empty())
		{
			const int f = q.front();
			q.pop();
			comp.push_back(f);
			for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(f)])
			{
				if (allowed.find(nb) == allowed.end())
				{
					continue;
				}
				const std::size_t nbIdx = static_cast<std::size_t>(faceToLocal[nb]);
				if (visited[nbIdx] != 0U)
				{
					continue;
				}
				visited[nbIdx] = 1U;
				q.push(nb);
			}
		}
		if (!comp.empty())
		{
			outComponents.push_back(std::move(comp));
		}
	}
}

void finalizeRing(
	const IndexedMeshLite& mesh,
	const std::vector<int>& faceIndices,
	const std::vector<Vec3>& faceCenters,
	const std::vector<double>& faceRadii,
	const std::vector<double>& faceSemiMajor,
	const std::vector<double>& faceSemiMinor,
	const std::vector<double>& faceRotationDeg,
	const int ringId,
	TubularCrossSectionRing& outRing)
{
	outRing.id = ringId;
	outRing.faceIndices = faceIndices;
	Vec3 centerSum{0.0, 0.0, 0.0};
	double radiusSum = 0.0;
	double semiMajorSum = 0.0;
	double semiMinorSum = 0.0;
	double rotSum = 0.0;
	for (const int f : faceIndices)
	{
		const std::size_t fi = static_cast<std::size_t>(f);
		centerSum = add(centerSum, faceCenters[fi]);
		radiusSum += faceRadii[fi];
		semiMajorSum += faceSemiMajor[fi];
		semiMinorSum += faceSemiMinor[fi];
		rotSum += faceRotationDeg[fi];
	}
	const double inv = 1.0 / static_cast<double>(faceIndices.size());
	const Vec3 center = scale(centerSum, inv);
	outRing.centerMm[0] = center.x;
	outRing.centerMm[1] = center.y;
	outRing.centerMm[2] = center.z;
	outRing.radiusMm = radiusSum * inv;
	outRing.semiMajorMm = semiMajorSum * inv;
	outRing.semiMinorMm = semiMinorSum * inv;
	outRing.sectionRotationDeg = rotSum * inv;
	outRing.aspectRatio = (outRing.semiMinorMm > 1e-6)
		? outRing.semiMajorMm / outRing.semiMinorMm : 1.0;
	outRing.axisHint = {0.0, 0.0, 1.0};
	(void)mesh;
}

void computeRingAxes(
	const std::map<std::pair<int, int>, int>& ringAdjacencyCount,
	std::vector<TubularCrossSectionRing>& inOutRings)
{
	std::vector<Vec3> axisAccum(inOutRings.size());
	for (const auto& kv : ringAdjacencyCount)
	{
		const int ra = kv.first.first;
		const int rb = kv.first.second;
		if (ra < 0 || rb < 0 || ra >= static_cast<int>(inOutRings.size()) || rb >= static_cast<int>(inOutRings.size()))
		{
			continue;
		}
		const TubularCrossSectionRing& a = inOutRings[static_cast<std::size_t>(ra)];
		const TubularCrossSectionRing& b = inOutRings[static_cast<std::size_t>(rb)];
		Vec3 dir{
			b.centerMm[0] - a.centerMm[0],
			b.centerMm[1] - a.centerMm[1],
			b.centerMm[2] - a.centerMm[2]};
		dir = normalizeVec3(dir);
		axisAccum[static_cast<std::size_t>(ra)] = add(axisAccum[static_cast<std::size_t>(ra)], dir);
		axisAccum[static_cast<std::size_t>(rb)] = add(axisAccum[static_cast<std::size_t>(rb)], scale(dir, -1.0));
	}
	for (std::size_t i = 0; i < inOutRings.size(); ++i)
	{
		Vec3 axis = normalizeVec3(axisAccum[i]);
		if (length(axisAccum[i]) < 1e-6)
		{
			axis = {0.0, 0.0, 1.0};
		}
		inOutRings[i].axisHint[0] = axis.x;
		inOutRings[i].axisHint[1] = axis.y;
		inOutRings[i].axisHint[2] = axis.z;
	}
}

bool markJunctionRings(
	const std::vector<TubularCrossSectionRing>& rings,
	const std::map<int, std::vector<int>>& ringNeighbors,
	const double junctionSpreadDeg,
	std::vector<uint8_t>& outIsJunctionRing)
{
	outIsJunctionRing.assign(rings.size(), 0U);
	bool any = false;
	for (std::size_t ri = 0; ri < rings.size(); ++ri)
	{
		const auto it = ringNeighbors.find(static_cast<int>(ri));
		if (it == ringNeighbors.end() || it->second.size() < 3U)
		{
			continue;
		}
		double maxSpread = 0.0;
		const Vec3 a0{
			rings[ri].axisHint[0],
			rings[ri].axisHint[1],
			rings[ri].axisHint[2]};
		for (const int nb : it->second)
		{
			if (nb < 0 || nb >= static_cast<int>(rings.size()))
			{
				continue;
			}
			const Vec3 an{
				rings[static_cast<std::size_t>(nb)].axisHint[0],
				rings[static_cast<std::size_t>(nb)].axisHint[1],
				rings[static_cast<std::size_t>(nb)].axisHint[2]};
			maxSpread = std::max(maxSpread, axisAngleDeg(a0, an));
		}
		if (maxSpread >= junctionSpreadDeg)
		{
			outIsJunctionRing[ri] = 1U;
			any = true;
		}
	}
	return any;
}

void mergeRingsIntoSegments(
	const IndexedMeshLite& mesh,
	const std::vector<TubularCrossSectionRing>& rings,
	const std::map<int, std::vector<int>>& ringNeighbors,
	const std::vector<uint8_t>& isJunctionRing,
	const double mergeAngleDeg,
	const int minSegmentFaces,
	std::vector<TubularPipeSegment>& outSegments,
	std::vector<int>& outFaceSegmentId,
	std::vector<int>& outFaceRingId,
	int& outJunctionFaceCount)
{
	outSegments.clear();
	outJunctionFaceCount = 0;
	outFaceSegmentId.assign(static_cast<std::size_t>(mesh.faceCount), kFaceUnassigned);
	outFaceRingId.assign(static_cast<std::size_t>(mesh.faceCount), kFaceUnassigned);

	const int ringCount = static_cast<int>(rings.size());
	if (ringCount <= 0)
	{
		return;
	}

	for (int ri = 0; ri < ringCount; ++ri)
	{
		for (const int f : rings[static_cast<std::size_t>(ri)].faceIndices)
		{
			outFaceRingId[static_cast<std::size_t>(f)] = ri;
		}
	}

	UnionFind uf(ringCount);
	for (int ri = 0; ri < ringCount; ++ri)
	{
		if (isJunctionRing[static_cast<std::size_t>(ri)] != 0U)
		{
			continue;
		}
		const auto it = ringNeighbors.find(ri);
		if (it == ringNeighbors.end())
		{
			continue;
		}
		const Vec3 axisA{
			rings[static_cast<std::size_t>(ri)].axisHint[0],
			rings[static_cast<std::size_t>(ri)].axisHint[1],
			rings[static_cast<std::size_t>(ri)].axisHint[2]};
		for (const int nb : it->second)
		{
			if (nb < 0 || nb >= ringCount || isJunctionRing[static_cast<std::size_t>(nb)] != 0U)
			{
				continue;
			}
			const Vec3 axisB{
				rings[static_cast<std::size_t>(nb)].axisHint[0],
				rings[static_cast<std::size_t>(nb)].axisHint[1],
				rings[static_cast<std::size_t>(nb)].axisHint[2]};
			if (axisAngleDeg(axisA, axisB) <= mergeAngleDeg)
			{
				uf.unite(ri, nb);
			}
		}
	}

	std::map<int, TubularPipeSegment> segmentMap;
	std::map<int, int> ringCountPerRoot;
	for (int ri = 0; ri < ringCount; ++ri)
	{
		if (isJunctionRing[static_cast<std::size_t>(ri)] != 0U)
		{
			for (const int f : rings[static_cast<std::size_t>(ri)].faceIndices)
			{
				outFaceSegmentId[static_cast<std::size_t>(f)] = kFaceJunction;
				++outJunctionFaceCount;
			}
			continue;
		}
		const int root = uf.find(ri);
		TubularPipeSegment& seg = segmentMap[root];
		for (const int f : rings[static_cast<std::size_t>(ri)].faceIndices)
		{
			seg.faceIndices.push_back(f);
		}
		seg.axisHint[0] += rings[static_cast<std::size_t>(ri)].axisHint[0];
		seg.axisHint[1] += rings[static_cast<std::size_t>(ri)].axisHint[1];
		seg.axisHint[2] += rings[static_cast<std::size_t>(ri)].axisHint[2];
		++ringCountPerRoot[root];
	}

	int nextId = 0;
	std::map<int, int> remap;
	for (auto& kv : segmentMap)
	{
		TubularPipeSegment seg = std::move(kv.second);
		if (static_cast<int>(seg.faceIndices.size()) < minSegmentFaces)
		{
			continue;
		}
		const int ringN = std::max(1, ringCountPerRoot[kv.first]);
		const double inv = 1.0 / static_cast<double>(ringN);
		Vec3 axis{seg.axisHint[0] * inv, seg.axisHint[1] * inv, seg.axisHint[2] * inv};
		axis = normalizeVec3(axis);
		seg.axisHint[0] = axis.x;
		seg.axisHint[1] = axis.y;
		seg.axisHint[2] = axis.z;
		remap[kv.first] = nextId;
		seg.id = nextId++;
		outSegments.push_back(std::move(seg));
	}

	for (int f = 0; f < mesh.faceCount; ++f)
	{
		if (outFaceSegmentId[static_cast<std::size_t>(f)] == kFaceJunction)
		{
			continue;
		}
		const int rid = outFaceRingId[static_cast<std::size_t>(f)];
		if (rid < 0)
		{
			continue;
		}
		const int root = uf.find(rid);
		const auto it = remap.find(root);
		if (it != remap.end())
		{
			outFaceSegmentId[static_cast<std::size_t>(f)] = it->second;
		}
	}
}

} // namespace

bool runPipeSegmentation(
	const IndexedMeshLite& mesh,
	const TubularGrindingParams& params,
	std::vector<TubularPipeSegment>& outSegments,
	std::vector<TubularCrossSectionRing>& outRings,
	std::vector<int>& outFaceSegmentId,
	int& outJunctionFaceCount,
	int& outRegionCountBeforeFilter,
	std::string* errMsg,
	std::vector<Vec3>* outFaceLocalAxes)
{
	outSegments.clear();
	outRings.clear();
	outJunctionFaceCount = 0;
	outRegionCountBeforeFilter = 0;
	outFaceSegmentId.assign(static_cast<std::size_t>(mesh.faceCount), kFaceUnassigned);
	// 初始化局部轴线输出
	if (outFaceLocalAxes)
	{
		outFaceLocalAxes->assign(static_cast<std::size_t>(mesh.faceCount), Vec3{0.0, 0.0, 0.0});
	}
	if (mesh.faceCount < 3)
	{
		if (errMsg)
		{
			*errMsg = "mesh too small for segmentation";
		}
		return false;
	}

	const int minRingFaces = std::max(2, static_cast<int>(params.minRingFaces));
	const int minSegmentFaces = std::max(2, static_cast<int>(params.minSegmentFaces));

	std::vector<Vec3> faceCenters(static_cast<std::size_t>(mesh.faceCount));
	std::vector<double> faceRadii(static_cast<std::size_t>(mesh.faceCount), 0.0);
	std::vector<uint8_t> faceValid(static_cast<std::size_t>(mesh.faceCount), 0U);
	std::vector<Vec3> validCenterPoints;
	validCenterPoints.reserve(static_cast<std::size_t>(mesh.faceCount));

	// 截面分析相关
	std::vector<double> faceSemiMajor(static_cast<std::size_t>(mesh.faceCount), 0.0);
	std::vector<double> faceSemiMinor(static_cast<std::size_t>(mesh.faceCount), 0.0);
	std::vector<double> faceRotationDeg(static_cast<std::size_t>(mesh.faceCount), 0.0);

	for (int f = 0; f < mesh.faceCount; ++f)
	{
		Vec3 center;
		double radius = 0.0;
		Vec3 localAxis{0.0, 0.0, 0.0};

		if (params.neighborhoodMode == NeighborhoodMode::Adaptive)
		{
			// 自适应邻域 + 加权PCA + 广义截面分析
			std::vector<double> geodesics;
			const std::vector<int> neighborhood = collectAdaptiveNeighborhood(
				mesh, f, params.geodesicRadiusMm, geodesics);
			if (neighborhood.size() < 3)
			{
				continue;
			}

			localAxis = computeLocalAxisFromWeightedPCA(mesh, f, neighborhood, geodesics);

			double semiMajor = 0.0, semiMinor = 0.0, rotDeg = 0.0;
			if (!analyzeCrossSection(mesh, neighborhood, localAxis, params.sectionFitMode,
				semiMajor, semiMinor, rotDeg, center))
			{
				continue;
			}

			radius = (semiMajor + semiMinor) * 0.5;
			faceSemiMajor[static_cast<std::size_t>(f)] = semiMajor;
			faceSemiMinor[static_cast<std::size_t>(f)] = semiMinor;
			faceRotationDeg[static_cast<std::size_t>(f)] = rotDeg;
		}
		else
		{
			// 原有逻辑：固定 2-hop 叉积
			localAxis = computeLocalAxisFromNormalCrossProducts(mesh, f, 2);
			if (!computeFaceCenterFromNormals(mesh, f, params.ringRayConvergenceEpsMm, center, radius))
			{
				continue;
			}
		}

		faceValid[static_cast<std::size_t>(f)] = 1U;
		faceCenters[static_cast<std::size_t>(f)] = center;
		faceRadii[static_cast<std::size_t>(f)] = radius;
		validCenterPoints.push_back(center);
		// 存储局部轴线
		if (outFaceLocalAxes)
		{
			(*outFaceLocalAxes)[static_cast<std::size_t>(f)] = localAxis;
		}
	}

	if (validCenterPoints.size() < static_cast<std::size_t>(minRingFaces))
	{
		if (errMsg)
		{
			*errMsg = "too few tubular faces for ring clustering";
		}
		return false;
	}

	double clusterEps = params.ringCenterClusterEpsMm;
	if (clusterEps <= 0.0)
	{
		clusterEps = estimateRingClusterEpsMm(mesh, validCenterPoints);
	}

	std::vector<Vec3> dbPoints = validCenterPoints;
	std::vector<int> validFaceIndex;
	validFaceIndex.reserve(validCenterPoints.size());
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		if (faceValid[static_cast<std::size_t>(f)] != 0U)
		{
			validFaceIndex.push_back(f);
		}
	}

	// 根据模式选择聚类算法
	std::vector<int> dbLabels;
	int clusterCount = 0;
	if (params.neighborhoodMode == NeighborhoodMode::Adaptive &&
		params.sectionFitMode != SectionFitMode::Circle)
	{
		// 扩展DBSCAN：空间坐标 + 截面特征联合聚类
		std::vector<double> validSemiMajor, validSemiMinor;
		validSemiMajor.reserve(validCenterPoints.size());
		validSemiMinor.reserve(validCenterPoints.size());
		for (const int fi : validFaceIndex)
		{
			validSemiMajor.push_back(faceSemiMajor[static_cast<std::size_t>(fi)]);
			validSemiMinor.push_back(faceSemiMinor[static_cast<std::size_t>(fi)]);
		}
		// 特征缩放：截面参数与空间坐标的权重比
		const double featureScale = 1.0 / std::max(1.0, clusterEps);
		clusterCount = runDbscanEnhanced(dbPoints, validSemiMajor, validSemiMinor,
			clusterEps, minRingFaces, featureScale, dbLabels);
	}
	else
	{
		clusterCount = runDbscan(dbPoints, clusterEps, minRingFaces, dbLabels);
	}
	outRegionCountBeforeFilter = clusterCount;

	std::map<int, std::vector<int>> clusterFaces;
	for (std::size_t i = 0; i < dbLabels.size(); ++i)
	{
		const int lbl = dbLabels[i];
		if (lbl < 0)
		{
			continue;
		}
		clusterFaces[lbl].push_back(validFaceIndex[i]);
	}

	outRings.clear();
	int nextRingId = 0;
	std::vector<int> faceRingLabel(static_cast<std::size_t>(mesh.faceCount), kFaceUnassigned);
	for (const auto& kv : clusterFaces)
	{
		std::vector<std::vector<int>> components;
		splitClusterByConnectivity(mesh, kv.second, components);
		for (const std::vector<int>& comp : components)
		{
			if (static_cast<int>(comp.size()) < minRingFaces)
			{
				continue;
			}
			TubularCrossSectionRing ring;
			finalizeRing(mesh, comp, faceCenters, faceRadii,
				faceSemiMajor, faceSemiMinor, faceRotationDeg,
				nextRingId, ring);
			for (const int f : comp)
			{
				faceRingLabel[static_cast<std::size_t>(f)] = nextRingId;
			}
			outRings.push_back(std::move(ring));
			++nextRingId;
		}
	}

	if (outRings.empty())
	{
		if (errMsg)
		{
			*errMsg = "no valid cross-section rings after clustering (clusters="
				+ std::to_string(clusterCount)
				+ " epsMm=" + std::to_string(clusterEps) + ")";
		}
		return false;
	}

	std::map<std::pair<int, int>, int> ringAdjacencyCount;
	std::map<int, std::vector<int>> ringNeighbors;
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		const int ra = faceRingLabel[static_cast<std::size_t>(f)];
		if (ra < 0)
		{
			continue;
		}
		for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(f)])
		{
			const int rb = faceRingLabel[static_cast<std::size_t>(nb)];
			if (rb < 0 || rb == ra)
			{
				continue;
			}
			const int lo = std::min(ra, rb);
			const int hi = std::max(ra, rb);
			++ringAdjacencyCount[{lo, hi}];
			ringNeighbors[ra].push_back(rb);
			ringNeighbors[rb].push_back(ra);
		}
	}
	for (auto& kv : ringNeighbors)
	{
		auto& nbs = kv.second;
		std::sort(nbs.begin(), nbs.end());
		nbs.erase(std::unique(nbs.begin(), nbs.end()), nbs.end());
	}

	computeRingAxes(ringAdjacencyCount, outRings);

	std::vector<uint8_t> isJunctionRing;
	markJunctionRings(outRings, ringNeighbors, params.junctionAxisSpreadDeg, isJunctionRing);

	std::vector<int> faceRingId;
	mergeRingsIntoSegments(
		mesh,
		outRings,
		ringNeighbors,
		isJunctionRing,
		params.axisMergeAngleDeg,
		minSegmentFaces,
		outSegments,
		outFaceSegmentId,
		faceRingId,
		outJunctionFaceCount);

	if (outSegments.empty())
	{
		const int relaxedMin = std::max(2, minSegmentFaces / 2);
		if (relaxedMin < minSegmentFaces)
		{
			mergeRingsIntoSegments(
				mesh,
				outRings,
				ringNeighbors,
				isJunctionRing,
				std::min(65.0, params.axisMergeAngleDeg * 1.35),
				relaxedMin,
				outSegments,
				outFaceSegmentId,
				faceRingId,
				outJunctionFaceCount);
		}
	}

	if (outSegments.empty())
	{
		if (errMsg)
		{
			*errMsg = "no valid pipe segments after ring chain merge (rings="
				+ std::to_string(static_cast<int>(outRings.size()))
				+ " junctionFaces=" + std::to_string(outJunctionFaceCount)
				+ " minRequired=" + std::to_string(minSegmentFaces) + ")";
		}
		return false;
	}
	return true;
}

} // namespace tg
} // namespace geoalgo
