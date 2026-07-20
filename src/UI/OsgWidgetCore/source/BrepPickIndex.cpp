/// @file BrepPickIndex.cpp
/// @brief BrepPickIndex 实现

#include "pch.h"

#include "BrepPickIndex.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include <BrepImportArtifacts.h>
#include <Discretize.h>
#include <ShapeHandle.h>
#include <ShapeQuery.h>
#include <Types.h>
#include <ViewTessellate.h>

namespace
{
double segmentDistancePx(double qx, double qy, double s0x, double s0y, double s1x, double s1y)
{
	const double segVx = s1x - s0x;
	const double segVy = s1y - s0y;
	const double offVx = qx - s0x;
	const double offVy = qy - s0y;
	const double len2Seg = segVx * segVx + segVy * segVy;
	if (len2Seg <= 1e-9)
	{
		return std::hypot(qx - s0x, qy - s0y);
	}
	const double dot = offVx * segVx + offVy * segVy;
	const double t = (std::max)(0.0, (std::min)(1.0, dot / len2Seg));
	const double projx = s0x + segVx * t;
	const double projy = s0y + segVy * t;
	return std::hypot(qx - projx, qy - projy);
}

double polylineDistancePx(const std::vector<float>& xyz, const osg::Matrixd& mvp, int viewportWidthPx,
						  int viewportHeightPx, double mouseX, double mouseY,
						  const std::function<bool(const geoalgo::Point3d&, osg::Vec3f&)>& modelPointToWorld)
{
	const std::size_t nPts = xyz.size() / 3U;
	if (nPts < 2U)
	{
		return (std::numeric_limits<double>::max)();
	}
	auto toScreen = [&](const geoalgo::Point3d& mp, double& sx, double& sy) -> bool
	{
		osg::Vec3f world;
		if (!modelPointToWorld(mp, world))
		{
			return false;
		}
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		if (clip.z() < -1.0 || clip.z() > 1.0)
		{
			return false;
		}
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidthPx);
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeightPx);
		return true;
	};

	double best = (std::numeric_limits<double>::max)();
	for (std::size_t i = 1; i < nPts; ++i)
	{
		const std::size_t i0 = (i - 1U) * 3U;
		const std::size_t i1 = i * 3U;
		const geoalgo::Point3d p0{xyz[i0], xyz[i0 + 1U], xyz[i0 + 2U]};
		const geoalgo::Point3d p1{xyz[i1], xyz[i1 + 1U], xyz[i1 + 2U]};
		double s0x = 0.0;
		double s0y = 0.0;
		double s1x = 0.0;
		double s1y = 0.0;
		if (!toScreen(p0, s0x, s0y) || !toScreen(p1, s1x, s1y))
		{
			continue;
		}
		best = (std::min)(best, segmentDistancePx(mouseX, mouseY, s0x, s0y, s1x, s1y));
	}
	return best;
}

void collectCandidateEdges(const BrepPickIndex& index, int hintFaceIndex, bool expandAdjacent,
						   std::vector<int>& outEdges)
{
	outEdges.clear();
	std::unordered_set<int> seen;
	const auto& faceEdges = index.faceEdgeIndices();
	const auto addFaceEdges = [&](int faceIdx)
	{
		if (faceIdx < 0 || static_cast<std::size_t>(faceIdx) >= faceEdges.size())
		{
			return;
		}
		for (int edgeIdx : faceEdges[static_cast<std::size_t>(faceIdx)])
		{
			if (seen.insert(edgeIdx).second)
			{
				outEdges.push_back(edgeIdx);
			}
		}
	};
	addFaceEdges(hintFaceIndex);
	if (!expandAdjacent || hintFaceIndex < 0 || static_cast<std::size_t>(hintFaceIndex) >= faceEdges.size())
	{
		return;
	}
	std::unordered_set<int> hintEdgeSet(faceEdges[static_cast<std::size_t>(hintFaceIndex)].begin(),
										faceEdges[static_cast<std::size_t>(hintFaceIndex)].end());
	for (int faceIdx = 0; faceIdx < static_cast<int>(faceEdges.size()); ++faceIdx)
	{
		if (faceIdx == hintFaceIndex)
		{
			continue;
		}
		for (int edgeIdx : faceEdges[static_cast<std::size_t>(faceIdx)])
		{
			if (hintEdgeSet.count(edgeIdx) != 0U)
			{
				addFaceEdges(faceIdx);
				break;
			}
		}
	}
}

} // namespace

bool BrepPickIndex::empty() const
{
	return m_triangleFaceIndex.empty();
}

std::size_t BrepPickIndex::triangleCount() const
{
	return m_triangleFaceIndex.size();
}

int BrepPickIndex::faceIndexForTriangle(unsigned triIndex) const
{
	if (triIndex >= m_triangleFaceIndex.size())
	{
		return -1;
	}
	return m_triangleFaceIndex[triIndex];
}

const std::vector<float>& BrepPickIndex::faceSoupModel(int faceIndex) const
{
	static const std::vector<float> kEmpty;
	if (faceIndex < 0 || static_cast<std::size_t>(faceIndex) >= m_faceSoupModel.size())
	{
		return kEmpty;
	}
	return m_faceSoupModel[static_cast<std::size_t>(faceIndex)];
}

const std::vector<float>& BrepPickIndex::edgePolylineModel(int edgeIndex) const
{
	static const std::vector<float> kEmpty;
	if (edgeIndex < 0 || static_cast<std::size_t>(edgeIndex) >= m_edgePolylinesModel.size())
	{
		return kEmpty;
	}
	return m_edgePolylinesModel[static_cast<std::size_t>(edgeIndex)];
}

const std::vector<std::vector<int>>& BrepPickIndex::faceEdgeIndices() const
{
	return m_faceEdgeIndices;
}

bool BrepPickIndex::build(const geoalgo::ShapeHandle& shape, std::string* errMsg)
{
	m_triangleFaceIndex.clear();
	m_faceSoupModel.clear();
	m_edgePolylinesModel.clear();
	m_faceEdgeIndices.clear();
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	std::vector<float> soup;
	if (!geoalgo::tessellateShapePerFaceMedium(shape, soup, m_triangleFaceIndex, &m_faceSoupModel, errMsg))
	{
		return false;
	}
	const std::size_t triCount = soup.size() / 9U;
	if (triCount != m_triangleFaceIndex.size())
	{
		if (errMsg)
		{
			*errMsg = "triangle face index size mismatch";
		}
		return false;
	}
	if (!geoalgo::collectShapeFaceEdgeIndices(shape, m_faceEdgeIndices, errMsg))
	{
		return false;
	}
	geoalgo::TessellateParams edgeParams;
	edgeParams.linearDeflectionMm = 0.05;
	edgeParams.angularDeflectionDeg = 1.0;
	edgeParams.linearDeflectionRelative = false;
	std::vector<geoalgo::Polyline3d> polylines;
	if (!geoalgo::discretizeShapeEdges(shape, edgeParams, polylines, errMsg))
	{
		return false;
	}
	m_edgePolylinesModel.clear();
	m_edgePolylinesModel.reserve(polylines.size());
	for (const geoalgo::Polyline3d& pl : polylines)
	{
		m_edgePolylinesModel.push_back(pl.xyz);
	}
	return true;
}

bool BrepPickIndex::buildFromArtifacts(const geoalgo::BrepImportArtifacts& artifacts, std::string* errMsg)
{
	m_triangleFaceIndex = artifacts.triangleFaceIndex;
	m_faceSoupModel = artifacts.faceSoups;
	m_edgePolylinesModel = artifacts.edgePolylines;
	m_faceEdgeIndices = artifacts.faceEdgeIndices;
	const std::size_t triCount = artifacts.displaySoup.size() / 9U;
	if (triCount != m_triangleFaceIndex.size())
	{
		if (errMsg)
		{
			*errMsg = "triangle face index size mismatch";
		}
		m_triangleFaceIndex.clear();
		m_faceSoupModel.clear();
		m_edgePolylinesModel.clear();
		m_faceEdgeIndices.clear();
		return false;
	}
	return true;
}

bool BrepPickIndex::pickEdgeByScreen(int hintFaceIndex, double mouseX, double mouseY, const osg::Matrixd& mvp,
									 int viewportWidthPx, int viewportHeightPx, double hitRadiusPx,
									 const std::function<bool(const geoalgo::Point3d&, osg::Vec3f&)>& modelPointToWorld,
									 int& outEdgeIndex, double& outDistancePx,
									 std::vector<osg::Vec3f>& outPolylineWorld) const
{
	outEdgeIndex = -1;
	outDistancePx = (std::numeric_limits<double>::max)();
	outPolylineWorld.clear();
	if (m_edgePolylinesModel.empty())
	{
		return false;
	}

	std::vector<int> candidateEdges;
	collectCandidateEdges(*this, hintFaceIndex, false, candidateEdges);
	auto scanCandidates = [&](const std::vector<int>& edges) -> bool
	{
		int bestEdge = -1;
		double bestDist = (std::numeric_limits<double>::max)();
		for (int edgeIdx : edges)
		{
			if (edgeIdx < 0 || static_cast<std::size_t>(edgeIdx) >= m_edgePolylinesModel.size())
			{
				continue;
			}
			const double dist =
				polylineDistancePx(m_edgePolylinesModel[static_cast<std::size_t>(edgeIdx)], mvp, viewportWidthPx,
								   viewportHeightPx, mouseX, mouseY, modelPointToWorld);
			if (dist < bestDist)
			{
				bestDist = dist;
				bestEdge = edgeIdx;
			}
		}
		if (bestEdge < 0 || bestDist > hitRadiusPx)
		{
			return false;
		}
		outEdgeIndex = bestEdge;
		outDistancePx = bestDist;
		const std::vector<float>& pl = m_edgePolylinesModel[static_cast<std::size_t>(bestEdge)];
		outPolylineWorld.reserve(pl.size() / 3U);
		for (std::size_t i = 0; i + 2U < pl.size(); i += 3U)
		{
			const geoalgo::Point3d mp{pl[i], pl[i + 1U], pl[i + 2U]};
			osg::Vec3f wp;
			if (modelPointToWorld(mp, wp))
			{
				outPolylineWorld.push_back(wp);
			}
		}
		return outPolylineWorld.size() >= 2U;
	};

	if (scanCandidates(candidateEdges))
	{
		return true;
	}
	collectCandidateEdges(*this, hintFaceIndex, true, candidateEdges);
	if (scanCandidates(candidateEdges))
	{
		return true;
	}
	candidateEdges.clear();
	candidateEdges.reserve(m_edgePolylinesModel.size());
	for (int edgeIdx = 0; edgeIdx < static_cast<int>(m_edgePolylinesModel.size()); ++edgeIdx)
	{
		candidateEdges.push_back(edgeIdx);
	}
	return scanCandidates(candidateEdges);
}
