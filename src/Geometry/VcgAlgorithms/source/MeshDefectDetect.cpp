#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "MeshDefectDetect.h"
#include "VcgMeshTypes.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <unordered_set>
#include <vector>

#include <vcg/complex/algorithms/update/curvature.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/space/triangle3.h>

namespace vcgalgo
{

namespace
{

using FacePointer = VcgMesh::FacePointer;
using CoordType = VcgMesh::CoordType;

double clamp01(const double v)
{
	return std::max(0.0, std::min(1.0, v));
}

double faceArea(const VcgMesh::FaceType& f)
{
	return static_cast<double>(vcg::DoubleArea(f)) * 0.5;
}

int faceIndexOf(const VcgMesh& mesh, const VcgMesh::FaceType& f)
{
	return static_cast<int>(&f - &mesh.face[0]);
}

double edgeLength(const CoordType& a, const CoordType& b)
{
	return static_cast<double>((b - a).Norm());
}

CoordType faceCentroid(const VcgMesh::FaceType& f)
{
	return (f.cP(0) + f.cP(1) + f.cP(2)) / 3.0;
}

CoordType normalized(const CoordType& v)
{
	const double len = static_cast<double>(v.Norm());
	if (len < 1e-12)
	{
		return CoordType(0.0, 0.0, 1.0);
	}
	return v / len;
}

// 面法向与邻接面法向夹角偏差
double faceNormalDeviation(const VcgMesh::FaceType& f)
{
	CoordType n = normalized(f.cN());
	double sumDot = 0.0;
	int count = 0;
	for (int e = 0; e < 3; ++e)
	{
		if (vcg::face::IsBorder(f, e))
		{
			continue;
		}
		const VcgMesh::FacePointer adj = f.FFp(e);
		if (!adj || adj->IsD())
		{
			continue;
		}
		sumDot += static_cast<double>(n * normalized(adj->cN()));
		++count;
	}
	if (count == 0)
	{
		return 0.0;
	}
	return std::max(0.0, 1.0 - sumDot / static_cast<double>(count));
}

double faceEdgeAspect(const VcgMesh::FaceType& f)
{
	double minE = std::numeric_limits<double>::max();
	double maxE = 0.0;
	for (int j = 0; j < 3; ++j)
	{
		const double len = edgeLength(f.cP(j), f.cP((j + 1) % 3));
		minE = std::min(minE, len);
		maxE = std::max(maxE, len);
	}
	return maxE / (minE + 1e-12);
}

double faceMaxEdgeOverMedian(const VcgMesh::FaceType& f, const double medianEdgeLen)
{
	double maxE = 0.0;
	for (int j = 0; j < 3; ++j)
	{
		maxE = std::max(maxE, edgeLength(f.cP(j), f.cP((j + 1) % 3)));
	}
	return maxE / (medianEdgeLen + 1e-12);
}

double vertexLocalCurvatureRatio(VcgMesh& mesh, VcgMesh::VertexType& v)
{
	const double self = static_cast<double>(v.Q());
	double ringSum = 0.0;
	int ringCount = 0;

	vcg::face::VFIterator<VcgMesh::FaceType> vfi(&v);
	for (; !vfi.End(); ++vfi)
	{
		VcgMesh::FaceType* face = vfi.F();
		if (!face || face->IsD())
		{
			continue;
		}
		for (int j = 0; j < 3; ++j)
		{
			VcgMesh::VertexType* nv = face->V(j);
			if (!nv || nv->IsD() || nv == &v)
			{
				continue;
			}
			ringSum += static_cast<double>(nv->Q());
			++ringCount;
		}
	}

	const double ringMean = ringCount > 0 ? ringSum / static_cast<double>(ringCount) : self;
	return self / (ringMean + 1e-12);
}

// sensitivity 越高（UI 百分比越大）阈值越松
double zScoreThresholdFromSensitivity(const double sensitivity)
{
	return std::max(1.8, 3.4 - sensitivity * 12.0);
}

struct LocalZResult
{
	double z = 0.0;
	bool significant = false;
};

// 相对 FF 邻域的 z-score；higherIsBad=true 表示值越大越异常
LocalZResult localZScore(
	const VcgMesh& mesh,
	const int fi,
	const std::vector<double>& signal,
	const double zThreshold,
	const bool higherIsBad)
{
	LocalZResult out;
	if (fi < 0 || fi >= static_cast<int>(signal.size()))
	{
		return out;
	}
	const double self = signal[static_cast<std::size_t>(fi)];

	std::vector<double> ring;
	ring.reserve(8);
	const VcgMesh::FaceType& f = mesh.face[static_cast<std::size_t>(fi)];
	for (int e = 0; e < 3; ++e)
	{
		if (vcg::face::IsBorder(f, e))
		{
			continue;
		}
		const FacePointer adj = f.FFp(e);
		if (!adj || adj->IsD())
		{
			continue;
		}
		const int ni = faceIndexOf(mesh, *adj);
		if (ni >= 0 && ni < static_cast<int>(signal.size()))
		{
			ring.push_back(signal[static_cast<std::size_t>(ni)]);
		}
	}
	if (ring.size() < 2U)
	{
		return out;
	}

	double mean = 0.0;
	for (const double v : ring)
	{
		mean += v;
	}
	mean /= static_cast<double>(ring.size());

	double var = 0.0;
	for (const double v : ring)
	{
		const double d = v - mean;
		var += d * d;
	}
	var /= static_cast<double>(ring.size());
	const double stddev = std::sqrt(var);
	if (stddev < 1e-9)
	{
		return out;
	}

	out.z = (self - mean) / stddev;
	if (higherIsBad)
	{
		out.significant = out.z >= zThreshold;
	}
	else
	{
		out.significant = out.z <= -zThreshold;
	}
	return out;
}

// 簇相对邻域的外凸高度（沿簇平均法向投影）
double clusterProtrusionHeight(
	const VcgMesh& mesh,
	const std::vector<int>& members,
	const std::unordered_set<int>& memberSet,
	const std::vector<CoordType>& faceCentroids,
	const std::vector<CoordType>& faceNormals)
{
	if (members.empty())
	{
		return 0.0;
	}

	CoordType clusterCenter(0.0, 0.0, 0.0);
	CoordType clusterNormal(0.0, 0.0, 0.0);
	for (const int fi : members)
	{
		clusterCenter += faceCentroids[static_cast<std::size_t>(fi)];
		clusterNormal += faceNormals[static_cast<std::size_t>(fi)];
	}
	const double invN = 1.0 / static_cast<double>(members.size());
	clusterCenter *= invN;
	clusterNormal = normalized(clusterNormal);

	CoordType baseline(0.0, 0.0, 0.0);
	int baselineCount = 0;
	for (const int fi : members)
	{
		const VcgMesh::FaceType& f = mesh.face[static_cast<std::size_t>(fi)];
		for (int e = 0; e < 3; ++e)
		{
			if (vcg::face::IsBorder(f, e))
			{
				continue;
			}
			const FacePointer adj = f.FFp(e);
			if (!adj || adj->IsD())
			{
				continue;
			}
			const int ni = faceIndexOf(mesh, *adj);
			if (ni < 0 || memberSet.count(ni) > 0)
			{
				continue;
			}
			baseline += faceCentroids[static_cast<std::size_t>(ni)];
			++baselineCount;
		}
	}
	if (baselineCount == 0)
	{
		return 0.0;
	}
	baseline /= static_cast<double>(baselineCount);

	const CoordType delta = clusterCenter - baseline;
	return std::max(0.0, static_cast<double>(delta * clusterNormal));
}

// 连通域过滤：过小、过大、凸起高度不足
void filterDefectClusters(
	VcgMesh& mesh,
	const int minClusterFaces,
	const double maxClusterAreaRatio,
	const double minProtrusionHeight,
	const double totalArea,
	const std::vector<CoordType>& faceCentroids,
	const std::vector<CoordType>& faceNormals,
	const std::vector<bool>& isProtrusion,
	const std::vector<bool>& isBoundarySpike,
	std::vector<bool>& defectMask)
{
	const int faceCount = static_cast<int>(mesh.face.size());
	const double maxClusterArea = totalArea * maxClusterAreaRatio;
	int nextCluster = 0;

	for (int fi = 0; fi < faceCount; ++fi)
	{
		if (mesh.face[static_cast<std::size_t>(fi)].IsD() || !defectMask[static_cast<std::size_t>(fi)])
		{
			continue;
		}

		std::vector<int> stack;
		stack.push_back(fi);
		defectMask[static_cast<std::size_t>(fi)] = false;

		std::vector<int> members;
		std::unordered_set<int> memberSet;
		double clusterArea = 0.0;

		while (!stack.empty())
		{
			const int cur = stack.back();
			stack.pop_back();
			members.push_back(cur);
			memberSet.insert(cur);
			clusterArea += faceArea(mesh.face[static_cast<std::size_t>(cur)]);

			VcgMesh::FaceType& cf = mesh.face[static_cast<std::size_t>(cur)];
			for (int j = 0; j < 3; ++j)
			{
				FacePointer nf = cf.FFp(j);
				if (!nf || nf->IsD())
				{
					continue;
				}
				const int ni = faceIndexOf(mesh, *nf);
				if (ni < 0 || !defectMask[static_cast<std::size_t>(ni)] || memberSet.count(ni) > 0)
				{
					continue;
				}
				defectMask[static_cast<std::size_t>(ni)] = false;
				stack.push_back(ni);
			}
		}

		bool keep = true;
		if (static_cast<int>(members.size()) < minClusterFaces)
		{
			keep = false;
		}
		if (keep && clusterArea > maxClusterArea)
		{
			keep = false;
		}
		bool needsHeightCheck = false;
		for (const int mi : members)
		{
			if (isProtrusion[static_cast<std::size_t>(mi)]
				|| isBoundarySpike[static_cast<std::size_t>(mi)])
			{
				needsHeightCheck = true;
				break;
			}
		}
		if (keep && needsHeightCheck && minProtrusionHeight > 0.0)
		{
			const double height = clusterProtrusionHeight(
				mesh, members, memberSet, faceCentroids, faceNormals);
			if (height < minProtrusionHeight)
			{
				keep = false;
			}
		}
		if (keep)
		{
			for (const int mi : members)
			{
				defectMask[static_cast<std::size_t>(mi)] = true;
			}
		}
		++nextCluster;
		(void)nextCluster;
	}
}

} // namespace

bool detectMeshDefects(
	const std::vector<float>& triangleSoup,
	DefectDetectReport& report,
	const DefectDetectParams& params,
	std::string* errMsg)
{
	report = DefectDetectReport{};

	if (triangleSoup.empty() || triangleSoup.size() % 9 != 0)
	{
		if (errMsg)
		{
			*errMsg = "detectMeshDefects: invalid triangle soup";
		}
		return false;
	}

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	const int totalFaces = static_cast<int>(mesh.fn);
	if (totalFaces <= 0)
	{
		if (errMsg)
		{
			*errMsg = "detectMeshDefects: mesh has no faces";
		}
		return false;
	}

	try
	{
		if (!internal::prepareMeshTopology(mesh, errMsg))
		{
			return false;
		}

		const double sensitivity = clamp01(params.sensitivity > 0.0 ? params.sensitivity : 0.08);
		const int minCluster = params.minClusterFaces > 0 ? params.minClusterFaces : 3;
		const double zThr = zScoreThresholdFromSensitivity(sensitivity);
		// 单簇面积不超过网格总面积的该比例，避免整片曲面被标为缺陷
		const double maxClusterAreaRatio = std::max(0.004, 0.025 - sensitivity * 0.12);
		const double localCurvRatioFloor = 1.08 + (1.0 - sensitivity) * 0.12;

		std::vector<double> faceQuality(static_cast<std::size_t>(totalFaces), 0.0);
		std::vector<double> normalDeviations(static_cast<std::size_t>(totalFaces), 0.0);
		std::vector<double> edgeAspects(static_cast<std::size_t>(totalFaces), 0.0);
		std::vector<double> longEdgeRatios(static_cast<std::size_t>(totalFaces), 0.0);
		std::vector<double> maxVertCurvatures(static_cast<std::size_t>(totalFaces), 0.0);
		std::vector<double> maxLocalCurvRatios(static_cast<std::size_t>(totalFaces), 0.0);
		std::vector<CoordType> faceCentroids(static_cast<std::size_t>(totalFaces));
		std::vector<CoordType> faceNormals(static_cast<std::size_t>(totalFaces));
		std::vector<double> vertLocalCurvRatio(mesh.vert.size(), 1.0);

		for (std::size_t vi = 0; vi < mesh.vert.size(); ++vi)
		{
			if (mesh.vert[vi].IsD())
			{
				continue;
			}
			vertLocalCurvRatio[vi] = vertexLocalCurvatureRatio(mesh, mesh.vert[vi]);
		}

		vcg::tri::UpdateCurvature<VcgMesh>::PerVertexAbsoluteMeanAndGaussian(mesh);
		auto khAttr = vcg::tri::Allocator<VcgMesh>::GetPerVertexAttribute<VcgMesh::ScalarType>(mesh, "KH");
		auto kgAttr = vcg::tri::Allocator<VcgMesh>::GetPerVertexAttribute<VcgMesh::ScalarType>(mesh, "KG");

		for (auto& v : mesh.vert)
		{
			if (v.IsD())
			{
				continue;
			}
			const double curvedness =
				std::abs(static_cast<double>(khAttr[v])) + std::abs(static_cast<double>(kgAttr[v]));
			v.Q() = curvedness;
		}

		std::vector<double> edgeLengths;
		edgeLengths.reserve(static_cast<std::size_t>(totalFaces) * 3U);
		double totalArea = 0.0;

		for (auto& f : mesh.face)
		{
			if (f.IsD())
			{
				continue;
			}
			const int fi = faceIndexOf(mesh, f);
			if (fi < 0 || fi >= totalFaces)
			{
				continue;
			}

			const double q = static_cast<double>(vcg::QualityMeanRatio(f.cP(0), f.cP(1), f.cP(2)));
			f.Q() = q;
			faceQuality[static_cast<std::size_t>(fi)] = q;
			normalDeviations[static_cast<std::size_t>(fi)] = faceNormalDeviation(f);
			edgeAspects[static_cast<std::size_t>(fi)] = faceEdgeAspect(f);
			faceCentroids[static_cast<std::size_t>(fi)] = faceCentroid(f);
			faceNormals[static_cast<std::size_t>(fi)] = normalized(f.cN());
			totalArea += faceArea(f);

			for (int j = 0; j < 3; ++j)
			{
				edgeLengths.push_back(edgeLength(f.cP(j), f.cP((j + 1) % 3)));
			}

			double maxVertCurv = 0.0;
			double maxLocalRatio = 1.0;
			for (int j = 0; j < 3; ++j)
			{
				auto* v = f.V(j);
				if (!v || v->IsD())
				{
					continue;
				}
				maxVertCurv = std::max(maxVertCurv, static_cast<double>(v->Q()));
				const std::size_t vIdx = static_cast<std::size_t>(v - &mesh.vert[0]);
				if (vIdx < vertLocalCurvRatio.size())
				{
					maxLocalRatio = std::max(maxLocalRatio, vertLocalCurvRatio[vIdx]);
				}
			}
			maxVertCurvatures[static_cast<std::size_t>(fi)] = maxVertCurv;
			maxLocalCurvRatios[static_cast<std::size_t>(fi)] = maxLocalRatio;
		}

		double medianEdgeLen = 0.0;
		if (!edgeLengths.empty())
		{
			std::vector<double> sorted = edgeLengths;
			const std::size_t mid = sorted.size() / 2U;
			std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(mid), sorted.end());
			medianEdgeLen = sorted[mid];
		}
		const double minProtrusionHeight = medianEdgeLen * (0.06 + (1.0 - sensitivity) * 0.18);

		for (auto& f : mesh.face)
		{
			if (f.IsD())
			{
				continue;
			}
			const int fi = faceIndexOf(mesh, f);
			if (fi < 0 || fi >= totalFaces)
			{
				continue;
			}
			longEdgeRatios[static_cast<std::size_t>(fi)] = faceMaxEdgeOverMedian(f, medianEdgeLen);
		}

		std::vector<bool> isNeedle(static_cast<std::size_t>(totalFaces), false);
		std::vector<bool> isProtrusion(static_cast<std::size_t>(totalFaces), false);
		std::vector<bool> isBoundarySpike(static_cast<std::size_t>(totalFaces), false);
		std::vector<double> defectScore(static_cast<std::size_t>(totalFaces), 0.0);

		for (auto& f : mesh.face)
		{
			if (f.IsD())
			{
				continue;
			}
			const int fi = faceIndexOf(mesh, f);
			if (fi < 0 || fi >= totalFaces)
			{
				continue;
			}

			const LocalZResult zQuality = localZScore(mesh, fi, faceQuality, zThr, false);
			const LocalZResult zNormalDev = localZScore(mesh, fi, normalDeviations, zThr, true);
			const LocalZResult zEdgeAspect = localZScore(mesh, fi, edgeAspects, zThr, true);
			const LocalZResult zLongEdge = localZScore(mesh, fi, longEdgeRatios, zThr, true);
			const LocalZResult zCurv = localZScore(mesh, fi, maxVertCurvatures, zThr, true);

			const double maxLocalRatio = maxLocalCurvRatios[static_cast<std::size_t>(fi)];
			const bool localCurvSpike = maxLocalRatio >= localCurvRatioFloor;

			bool hasBorderVert = false;
			bool isBorderFace = false;
			for (int j = 0; j < 3; ++j)
			{
				auto* v = f.V(j);
				if (v && !v->IsD() && v->IsB())
				{
					hasBorderVert = true;
				}
				if (vcg::face::IsBorder(f, j))
				{
					isBorderFace = true;
				}
			}

			// 针状：邻域内质量偏低或边长纵横比异常（局部 z-score）
			if (params.detectNeedle
				&& (zQuality.significant || zEdgeAspect.significant || zLongEdge.significant
					|| edgeAspects[static_cast<std::size_t>(fi)] >= 8.0))
			{
				isNeedle[static_cast<std::size_t>(fi)] = true;
				defectScore[static_cast<std::size_t>(fi)] = std::max(
					defectScore[static_cast<std::size_t>(fi)],
					std::max({
						std::abs(zQuality.z) / (zThr + 1e-9),
						zEdgeAspect.z / (zThr + 1e-9),
						zLongEdge.z / (zThr + 1e-9) }));
			}

			// 突起：邻域法向/曲率 z-score 或局部曲率倍率，不再用全局百分位
			const bool protrusionByNormal = zNormalDev.significant;
			const bool protrusionByCurv = zCurv.significant && localCurvSpike;
			const bool protrusionByLocal = localCurvSpike && maxLocalRatio >= (localCurvRatioFloor + 0.08);
			const bool protrusionByEdge = zLongEdge.significant && (protrusionByNormal || protrusionByLocal);
			if (params.detectProtrusion
				&& (protrusionByNormal || protrusionByCurv || protrusionByLocal || protrusionByEdge))
			{
				isProtrusion[static_cast<std::size_t>(fi)] = true;
				defectScore[static_cast<std::size_t>(fi)] = std::max(
					defectScore[static_cast<std::size_t>(fi)],
					std::max({
						zNormalDev.z / (zThr + 1e-9),
						zCurv.z / (zThr + 1e-9),
						(maxLocalRatio - 1.0) / (localCurvRatioFloor - 1.0 + 1e-9),
						zLongEdge.z / (zThr + 1e-9) }));
			}

			if (params.detectBoundarySpike && hasBorderVert && isBorderFace
				&& (zNormalDev.significant || protrusionByLocal || zCurv.significant))
			{
				isBoundarySpike[static_cast<std::size_t>(fi)] = true;
				defectScore[static_cast<std::size_t>(fi)] = std::max(
					defectScore[static_cast<std::size_t>(fi)],
					std::max({
						zNormalDev.z / (zThr + 1e-9),
						(maxLocalRatio - 1.0) / (localCurvRatioFloor - 1.0 + 1e-9),
						zCurv.z / (zThr + 1e-9) }));
			}
		}

		std::vector<bool> defectMask(static_cast<std::size_t>(totalFaces), false);
		for (int fi = 0; fi < totalFaces; ++fi)
		{
			defectMask[static_cast<std::size_t>(fi)] =
				isNeedle[static_cast<std::size_t>(fi)]
				|| isProtrusion[static_cast<std::size_t>(fi)]
				|| isBoundarySpike[static_cast<std::size_t>(fi)];
		}

		filterDefectClusters(
			mesh,
			minCluster,
			maxClusterAreaRatio,
			minProtrusionHeight,
			totalArea,
			faceCentroids,
			faceNormals,
			isProtrusion,
			isBoundarySpike,
			defectMask);

		report.totalFaces = totalFaces;
		report.defects.clear();
		report.defects.reserve(static_cast<std::size_t>(totalFaces) / 20U);

		double defectArea = 0.0;
		for (int fi = 0; fi < totalFaces; ++fi)
		{
			if (!defectMask[static_cast<std::size_t>(fi)])
			{
				continue;
			}

			MeshDefectKind kind = MeshDefectKind::Unknown;
			if (isBoundarySpike[static_cast<std::size_t>(fi)])
			{
				kind = MeshDefectKind::BoundarySpike;
				++report.boundarySpikeCount;
			}
			else if (isProtrusion[static_cast<std::size_t>(fi)])
			{
				kind = MeshDefectKind::HighCurvatureProtrusion;
				++report.protrusionCount;
			}
			else if (isNeedle[static_cast<std::size_t>(fi)])
			{
				kind = MeshDefectKind::NeedleTriangle;
				++report.needleCount;
			}

			defectArea += faceArea(mesh.face[static_cast<std::size_t>(fi)]);

			MeshDefectFace item;
			item.faceIndex = fi;
			item.kind = kind;
			item.score = clamp01(defectScore[static_cast<std::size_t>(fi)]);
			report.defects.push_back(item);
		}

		report.defectFaceCount = static_cast<int>(report.defects.size());
		report.defectAreaRatio = totalArea > 0.0 ? defectArea / totalArea : 0.0;

		std::sort(report.defects.begin(), report.defects.end(),
			[](const MeshDefectFace& a, const MeshDefectFace& b) { return a.score > b.score; });

		return true;
	}
	catch (const std::exception& e)
	{
		if (errMsg)
		{
			*errMsg = e.what();
		}
		return false;
	}
}

} // namespace vcgalgo
