/// @file FaceSectionDiscretize.cpp
/// @brief FaceSectionDiscretize 实现

#include "detail/FaceSectionDiscretize.h"

#include "FeatureDiscretizeParamUtils.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeFrame.h"
#include "detail/OccIncludes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <BRepAdaptor_CompCurve.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <gp_Ax1.hxx>
#include <gp_Quaternion.hxx>

namespace geoalgo
{
namespace
{
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kSeamTol = Precision::Confusion() * 100.0;
constexpr double kConnectTol = Precision::Confusion() * 10.0;

enum class TrajConnectMode
{
	Z,
	Bow,
	Zhi
};

Point3d pointFromGp(const gp_Pnt& p)
{
	return Point3d{p.X(), p.Y(), p.Z()};
}

Vec3d vecFromGp(const gp_Vec& v)
{
	return Vec3d{v.X(), v.Y(), v.Z()};
}

TrajConnectMode parseTrajConnectMode(const std::string& mode)
{
	if (mode == "Z")
	{
		return TrajConnectMode::Z;
	}
	if (mode == "Zhi")
	{
		return TrajConnectMode::Zhi;
	}
	return TrajConnectMode::Bow;
}

gp_Dir computeSectionNormal(const double rxDeg, const double ryDeg)
{
	gp_Quaternion quat;
	quat.SetEulerAngles(gp_Intrinsic_XYZ, rxDeg * kDeg2Rad, ryDeg * kDeg2Rad, 0.0);
	gp_Vec axis;
	Standard_Real angle = 0.0;
	quat.GetVectorAndAngle(axis, angle);
	gp_Dir dir(0.0, 0.0, 1.0);
	if (angle > Precision::Angular())
	{
		dir.Rotate(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), axis), angle);
	}
	return dir;
}

void appendSectionEdges(const TopoDS_Shape& sectionResult, const TopoDS_Face& refFace,
						std::vector<std::pair<TopoDS_Edge, TopoDS_Face>>& outEdges)
{
	for (TopExp_Explorer ex(sectionResult, TopAbs_EDGE); ex.More(); ex.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
		if (!edge.IsNull() && !BRep_Tool::Degenerated(edge))
		{
			outEdges.emplace_back(edge, refFace);
		}
	}
}

void sectionShapeWithPlane(const TopoDS_Shape& target, const TopoDS_Face& refFace, const gp_Pln& pln,
						   std::vector<std::pair<TopoDS_Edge, TopoDS_Face>>& outEdges)
{
	constexpr double planeSize = 1e6;
	const TopoDS_Face planeFace = BRepBuilderAPI_MakeFace(pln, -planeSize, planeSize, -planeSize, planeSize).Face();
	BRepAlgoAPI_Section sec(planeFace, target, Standard_False);
	sec.ComputePCurveOn1(Standard_False);
	sec.ComputePCurveOn2(Standard_False);
	sec.Approximation(Standard_True);
	sec.Build();
	if (!sec.IsDone())
	{
		return;
	}
	appendSectionEdges(sec.Shape(), refFace, outEdges);
}

void sectionFaceWithPlane(const TopoDS_Face& face, const gp_Pln& pln,
						  std::vector<std::pair<TopoDS_Edge, TopoDS_Face>>& outEdges)
{
	sectionShapeWithPlane(face, face, pln, outEdges);
}

bool discretizeEdgeUniform(const TopoDS_Edge& edge, int segmentCount, std::vector<Point3d>& pts)
{
	if (segmentCount < 2)
	{
		segmentCount = 2;
	}
	BRepAdaptor_Curve adaptor(edge);
	GCPnts_UniformAbscissa discretizer(adaptor, segmentCount + 1);
	if (!discretizer.IsDone() || discretizer.NbPoints() < 2)
	{
		return false;
	}
	pts.clear();
	pts.reserve(static_cast<std::size_t>(discretizer.NbPoints()));
	for (int i = 1; i <= discretizer.NbPoints(); ++i)
	{
		pts.push_back(pointFromGp(adaptor.Value(discretizer.Parameter(i))));
	}
	return true;
}

bool discretizeEdgeChordHeight(const TopoDS_Edge& edge, const double chordHeight, std::vector<Point3d>& pts)
{
	if (chordHeight <= 0.0)
	{
		return false;
	}
	BRepAdaptor_Curve adaptor(edge);
	GCPnts_UniformDeflection discretizer(adaptor, chordHeight);
	if (!discretizer.IsDone() || discretizer.NbPoints() < 2)
	{
		return false;
	}
	pts.clear();
	pts.reserve(static_cast<std::size_t>(discretizer.NbPoints()));
	for (int i = 1; i <= discretizer.NbPoints(); ++i)
	{
		pts.push_back(pointFromGp(adaptor.Value(discretizer.Parameter(i))));
	}
	return true;
}

void getEdgeEndpoints(const TopoDS_Edge& edge, gp_Pnt& pFirst, gp_Pnt& pLast)
{
	BRepAdaptor_Curve adaptor(edge);
	pFirst = adaptor.Value(adaptor.FirstParameter());
	pLast = adaptor.Value(adaptor.LastParameter());
}

bool pointsCoincident(const gp_Pnt& a, const gp_Pnt& b, const double tol)
{
	return a.Distance(b) <= tol;
}

bool discretizeWireUniform(const TopoDS_Wire& wire, int segmentCount, std::vector<Point3d>& pts)
{
	if (wire.IsNull())
	{
		return false;
	}
	if (segmentCount < 2)
	{
		segmentCount = 2;
	}
	BRepAdaptor_CompCurve comp(wire);
	GCPnts_UniformAbscissa discretizer(comp, segmentCount + 1);
	if (!discretizer.IsDone() || discretizer.NbPoints() < 2)
	{
		return false;
	}
	pts.clear();
	pts.reserve(static_cast<std::size_t>(discretizer.NbPoints()));
	for (int i = 1; i <= discretizer.NbPoints(); ++i)
	{
		pts.push_back(pointFromGp(comp.Value(discretizer.Parameter(i))));
	}
	return true;
}

bool discretizeWireChordHeight(const TopoDS_Wire& wire, const double chordHeight, std::vector<Point3d>& pts)
{
	if (wire.IsNull() || chordHeight <= 0.0)
	{
		return false;
	}
	BRepAdaptor_CompCurve comp(wire);
	GCPnts_UniformDeflection discretizer(comp, chordHeight);
	if (!discretizer.IsDone() || discretizer.NbPoints() < 2)
	{
		return false;
	}
	pts.clear();
	pts.reserve(static_cast<std::size_t>(discretizer.NbPoints()));
	for (int i = 1; i <= discretizer.NbPoints(); ++i)
	{
		pts.push_back(pointFromGp(comp.Value(discretizer.Parameter(i))));
	}
	return true;
}

void connectSectionEdgesToWires(const std::vector<std::pair<TopoDS_Edge, TopoDS_Face>>& edgesWithFace, const double tol,
								std::vector<TopoDS_Wire>& outWires, std::size_t& outConnectedEdgeCount)
{
	outWires.clear();
	outConnectedEdgeCount = 0;
	if (edgesWithFace.empty())
	{
		return;
	}

	struct EdgeEntry
	{
		TopoDS_Edge edge;
		gp_Pnt pFirst;
		gp_Pnt pLast;
	};

	std::vector<EdgeEntry> entries;
	entries.reserve(edgesWithFace.size());
	for (const auto& ef : edgesWithFace)
	{
		EdgeEntry entry;
		entry.edge = ef.first;
		getEdgeEndpoints(entry.edge, entry.pFirst, entry.pLast);
		entries.push_back(entry);
	}

	std::vector<bool> used(entries.size(), false);

	const auto extendForward =
		[&](const gp_Pnt& startEnd, std::vector<TopoDS_Edge>& chain, gp_Pnt& chainEnd, std::vector<bool>& localUsed)
	{
		gp_Pnt end = startEnd;
		bool extended = true;
		while (extended)
		{
			extended = false;
			for (std::size_t i = 0; i < entries.size(); ++i)
			{
				if (localUsed[i])
				{
					continue;
				}
				if (pointsCoincident(end, entries[i].pFirst, tol))
				{
					chain.push_back(entries[i].edge);
					end = entries[i].pLast;
					localUsed[i] = true;
					extended = true;
					break;
				}
				if (pointsCoincident(end, entries[i].pLast, tol))
				{
					chain.push_back(TopoDS::Edge(entries[i].edge.Reversed()));
					end = entries[i].pFirst;
					localUsed[i] = true;
					extended = true;
					break;
				}
			}
		}
		chainEnd = end;
	};

	const auto extendBackward =
		[&](const gp_Pnt& startFront, std::vector<TopoDS_Edge>& chain, gp_Pnt& chainStart, std::vector<bool>& localUsed)
	{
		gp_Pnt front = startFront;
		bool extended = true;
		while (extended)
		{
			extended = false;
			for (std::size_t i = 0; i < entries.size(); ++i)
			{
				if (localUsed[i])
				{
					continue;
				}
				if (pointsCoincident(front, entries[i].pLast, tol))
				{
					chain.insert(chain.begin(), entries[i].edge);
					front = entries[i].pFirst;
					localUsed[i] = true;
					extended = true;
					break;
				}
				if (pointsCoincident(front, entries[i].pFirst, tol))
				{
					chain.insert(chain.begin(), TopoDS::Edge(entries[i].edge.Reversed()));
					front = entries[i].pLast;
					localUsed[i] = true;
					extended = true;
					break;
				}
			}
		}
		chainStart = front;
	};

	while (true)
	{
		std::size_t startIdx = entries.size();
		for (std::size_t i = 0; i < entries.size(); ++i)
		{
			if (!used[i])
			{
				startIdx = i;
				break;
			}
		}
		if (startIdx == entries.size())
		{
			break;
		}

		std::size_t bestCount = 0;
		std::vector<TopoDS_Edge> bestChain;
		std::vector<bool> bestUsed = used;

		for (int orient = 0; orient < 2; ++orient)
		{
			std::vector<bool> localUsed = used;
			std::vector<TopoDS_Edge> chain;
			const bool reverseStart = (orient == 1);
			localUsed[startIdx] = true;

			if (reverseStart)
			{
				chain.push_back(TopoDS::Edge(entries[startIdx].edge.Reversed()));
			}
			else
			{
				chain.push_back(entries[startIdx].edge);
			}

			gp_Pnt chainStart = reverseStart ? entries[startIdx].pLast : entries[startIdx].pFirst;
			gp_Pnt chainEnd = reverseStart ? entries[startIdx].pFirst : entries[startIdx].pLast;
			extendForward(chainEnd, chain, chainEnd, localUsed);
			extendBackward(chainStart, chain, chainStart, localUsed);

			std::size_t count = 0;
			for (const bool u : localUsed)
			{
				if (u)
				{
					++count;
				}
			}
			if (count > bestCount)
			{
				bestCount = count;
				bestChain = std::move(chain);
				bestUsed = std::move(localUsed);
			}
		}

		used = std::move(bestUsed);
		outConnectedEdgeCount += bestCount;

		BRepBuilderAPI_MakeWire wireMaker;
		for (const TopoDS_Edge& e : bestChain)
		{
			wireMaker.Add(e);
		}
		if (wireMaker.IsDone())
		{
			outWires.push_back(wireMaker.Wire());
		}
	}
}

bool computeFaceNormalAtUv(const TopoDS_Face& face, const Handle(Geom_Surface) & surf, const TopLoc_Location& faceLoc,
						   const Standard_Real u, const Standard_Real v, gp_Vec& outNormal)
{
	gp_Pnt pOnSurf;
	gp_Vec du;
	gp_Vec dv;
	surf->D1(u, v, pOnSurf, du, dv);
	gp_Vec nLocal = du.Crossed(dv);
	const double mag2 = nLocal.SquareMagnitude();
	if (mag2 <= Precision::SquareConfusion())
	{
		return false;
	}
	nLocal /= std::sqrt(mag2);
	if (face.Orientation() == TopAbs_REVERSED)
	{
		nLocal.Reverse();
	}
	const gp_Trsf& toGlobal = faceLoc.Transformation();
	nLocal.Transform(toGlobal);
	outNormal = nLocal;
	if (outNormal.Magnitude() > Precision::Confusion())
	{
		outNormal.Normalize();
	}
	return true;
}

bool tryFaceNormalAtPoint(const TopoDS_Face& face, const gp_Pnt& ptWorld, double& inOutBestDist,
						  gp_Vec& inOutBestNormal)
{
	TopLoc_Location faceLoc;
	const Handle(Geom_Surface) surf = BRep_Tool::Surface(face, faceLoc);
	if (surf.IsNull())
	{
		return false;
	}

	const gp_Trsf& toGlobal = faceLoc.Transformation();
	gp_Trsf toLocal = toGlobal;
	toLocal.Invert();

	gp_Pnt ptLocal = ptWorld;
	ptLocal.Transform(toLocal);

	GeomAPI_ProjectPointOnSurf proj(ptLocal, surf);
	if (!proj.IsDone() || proj.NbPoints() < 1)
	{
		return false;
	}

	Standard_Real u = 0.0;
	Standard_Real v = 0.0;
	proj.LowerDistanceParameters(u, v);

	gp_Pnt pOnSurf;
	surf->D0(u, v, pOnSurf);
	pOnSurf.Transform(toGlobal);
	const double dist = ptWorld.Distance(pOnSurf);

	gp_Vec n;
	if (!computeFaceNormalAtUv(face, surf, faceLoc, u, v, n))
	{
		return false;
	}

	if (dist < inOutBestDist)
	{
		inOutBestDist = dist;
		inOutBestNormal = n;
		return true;
	}
	return false;
}

bool getBestFaceNormalAtPoint(const std::vector<TopoDS_Face>& faces, const gp_Pnt& ptWorld, gp_Vec& outNormal)
{
	const double maxDist = Precision::Confusion() * 100.0 * 10.0;

	double bestDist = std::numeric_limits<double>::max();
	gp_Vec bestNormal;
	bool found = false;

	for (const TopoDS_Face& face : faces)
	{
		found = tryFaceNormalAtPoint(face, ptWorld, bestDist, bestNormal) || found;
	}

	if (!found || bestDist > maxDist)
	{
		return false;
	}

	outNormal = bestNormal;
	return true;
}

bool filterLayerPointsByIndex(std::vector<Point3d>& pts, int startOrd, int endOrd)
{
	if (startOrd == 0 && endOrd == 0)
	{
		return !pts.empty();
	}
	if (pts.empty())
	{
		return false;
	}

	const int pointCount = static_cast<int>(pts.size());
	int start = (startOrd <= 0) ? 1 : startOrd;
	int end = (endOrd <= 0) ? pointCount : endOrd;
	if (start < 1)
	{
		start = 1;
	}
	if (end > pointCount)
	{
		end = pointCount;
	}
	if (start > end)
	{
		std::swap(start, end);
	}

	if (start == 1 && end == pointCount)
	{
		return pointCount >= 2;
	}

	std::vector<Point3d> filtered;
	filtered.reserve(static_cast<std::size_t>(end - start + 1));
	for (int i = start; i <= end; ++i)
	{
		filtered.push_back(pts[static_cast<std::size_t>(i - 1)]);
	}
	pts = std::move(filtered);
	return pts.size() >= 2;
}

bool appendPointDedup(RawPath& path, const Point3d& pos, const Vec3d& normal, const double seamTol)
{
	if (!path.points.empty())
	{
		const Point3d& last = path.points.back().positionMm;
		const double dx = pos.x - last.x;
		const double dy = pos.y - last.y;
		const double dz = pos.z - last.z;
		if (std::sqrt(dx * dx + dy * dy + dz * dz) <= seamTol)
		{
			return false;
		}
	}
	RawPathPoint rp;
	rp.positionMm = pos;
	if (std::abs(normal.x) + std::abs(normal.y) + std::abs(normal.z) > 1e-9)
	{
		rp.normal = normal;
		rp.hasNormal = true;
	}
	path.points.push_back(rp);
	return true;
}

void appendCuttingPointsToPath(const std::vector<Point3d>& pts, const std::vector<TopoDS_Face>& faces, RawPath& path,
							   const double seamTol)
{
	for (const Point3d& pt : pts)
	{
		bool gotNormal = false;
		const Vec3d normal =
			detail::storedFaceNormal(faces, pt, detail::FaceNormalConvention::LineReverseFace, &gotNormal);
		(void)appendPointDedup(path, pt, gotNormal ? normal : Vec3d{}, seamTol);
	}
}

double pointDist(const Point3d& a, const Point3d& b)
{
	const double dx = a.x - b.x;
	const double dy = a.y - b.y;
	const double dz = a.z - b.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void appendSegmentPointsToLayer(std::vector<Point3d>& layerPts, const std::vector<Point3d>& segment,
								const double seamTol, const bool zhiJumpOnly)
{
	if (segment.size() < 2)
	{
		return;
	}
	if (layerPts.empty())
	{
		layerPts = segment;
		return;
	}

	std::vector<Point3d> oriented = segment;
	const Point3d& tail = layerPts.back();
	const double distToStart = pointDist(tail, segment.front());
	const double distToEnd = pointDist(tail, segment.back());
	if (distToEnd + 1e-9 < distToStart)
	{
		std::reverse(oriented.begin(), oriented.end());
	}

	const double connectDist = pointDist(tail, oriented.front());
	const bool physicallyConnected = connectDist <= seamTol;

	if (zhiJumpOnly && !physicallyConnected)
	{
		if (connectDist > seamTol)
		{
			layerPts.push_back(oriented.front());
		}
		return;
	}

	for (const Point3d& p : oriented)
	{
		if (!layerPts.empty() && pointDist(p, layerPts.back()) <= seamTol)
		{
			continue;
		}
		layerPts.push_back(p);
	}
}

struct LayerSegment
{
	std::vector<Point3d> pts;
};

std::size_t segmentSortKeyIndex(const std::vector<LayerSegment>& segments)
{
	std::size_t bestIdx = 0;
	gp_Pnt bestKey(0.0, 0.0, 0.0);
	bool hasKey = false;
	for (std::size_t i = 0; i < segments.size(); ++i)
	{
		if (segments[i].pts.size() < 2)
		{
			continue;
		}
		const gp_Pnt key(segments[i].pts.front().x, segments[i].pts.front().y, segments[i].pts.front().z);
		if (!hasKey)
		{
			bestIdx = i;
			bestKey = key;
			hasKey = true;
			continue;
		}
		if (std::abs(key.X() - bestKey.X()) > Precision::Confusion())
		{
			if (key.X() < bestKey.X())
			{
				bestIdx = i;
				bestKey = key;
			}
			continue;
		}
		if (std::abs(key.Y() - bestKey.Y()) > Precision::Confusion())
		{
			if (key.Y() < bestKey.Y())
			{
				bestIdx = i;
				bestKey = key;
			}
			continue;
		}
		if (key.Z() < bestKey.Z())
		{
			bestIdx = i;
			bestKey = key;
		}
	}
	return bestIdx;
}

bool stitchLayerSegments(std::vector<LayerSegment>& segments, const double seamTol, const bool zhiMode,
						 std::vector<Point3d>& outLayerPts)
{
	outLayerPts.clear();
	if (segments.empty())
	{
		return false;
	}

	std::vector<bool> used(segments.size(), false);
	const std::size_t seedIdx = segmentSortKeyIndex(segments);
	if (segments[seedIdx].pts.size() < 2)
	{
		return false;
	}

	used[seedIdx] = true;
	outLayerPts = segments[seedIdx].pts;

	while (true)
	{
		std::size_t bestIdx = segments.size();
		double bestDist = std::numeric_limits<double>::max();
		const Point3d& tail = outLayerPts.back();

		for (std::size_t i = 0; i < segments.size(); ++i)
		{
			if (used[i] || segments[i].pts.size() < 2)
			{
				continue;
			}
			const double distStart = pointDist(tail, segments[i].pts.front());
			const double distEnd = pointDist(tail, segments[i].pts.back());
			if (distStart < bestDist)
			{
				bestDist = distStart;
				bestIdx = i;
			}
			if (distEnd < bestDist)
			{
				bestDist = distEnd;
				bestIdx = i;
			}
		}
		if (bestIdx == segments.size())
		{
			break;
		}

		used[bestIdx] = true;
		appendSegmentPointsToLayer(outLayerPts, segments[bestIdx].pts, seamTol, zhiMode && bestDist > seamTol);
	}

	return outLayerPts.size() >= 2;
}

bool discretizeEdgeToSegment(const TopoDS_Edge& edge, const bool useChordHeight, const int uniformSegs,
							 const double chordHeight, const int layerKeepStart, const int layerKeepEnd,
							 LayerSegment& segment)
{
	std::vector<Point3d> pts;
	const bool ok = useChordHeight ? discretizeEdgeChordHeight(edge, chordHeight, pts)
								   : discretizeEdgeUniform(edge, uniformSegs, pts);
	if (!ok || !filterLayerPointsByIndex(pts, layerKeepStart, layerKeepEnd))
	{
		return false;
	}
	segment.pts = std::move(pts);
	return true;
}

bool discretizeWireToSegment(const TopoDS_Wire& wire, const bool useChordHeight, const int uniformSegs,
							 const double chordHeight, const int layerKeepStart, const int layerKeepEnd,
							 LayerSegment& segment)
{
	std::vector<Point3d> pts;
	const bool ok = useChordHeight ? discretizeWireChordHeight(wire, chordHeight, pts)
								   : discretizeWireUniform(wire, uniformSegs, pts);
	if (!ok || !filterLayerPointsByIndex(pts, layerKeepStart, layerKeepEnd))
	{
		return false;
	}
	segment.pts = std::move(pts);
	return true;
}

bool discretizeLayer(const std::vector<std::pair<TopoDS_Edge, TopoDS_Face>>& edgesWithFace, const bool useChordHeight,
					 const int uniformSegs, const double chordHeight, const int layerKeepStart, const int layerKeepEnd,
					 const bool zhiMode, const bool doReverse, const std::vector<TopoDS_Face>& faces, RawPath& path)
{
	std::vector<TopoDS_Wire> wires;
	std::size_t connectedEdgeCount = 0;
	connectSectionEdgesToWires(edgesWithFace, kConnectTol, wires, connectedEdgeCount);

	std::vector<LayerSegment> segments;
	segments.reserve(wires.size() + edgesWithFace.size());

	for (const TopoDS_Wire& wire : wires)
	{
		LayerSegment segment;
		if (discretizeWireToSegment(wire, useChordHeight, uniformSegs, chordHeight, layerKeepStart, layerKeepEnd,
									segment))
		{
			segments.push_back(std::move(segment));
		}
	}

	if (connectedEdgeCount < edgesWithFace.size())
	{
		std::vector<bool> edgeInWire(edgesWithFace.size(), false);
		if (!wires.empty())
		{
			for (const TopoDS_Wire& wire : wires)
			{
				for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next())
				{
					const TopoDS_Edge wireEdge = TopoDS::Edge(ex.Current());
					for (std::size_t ei = 0; ei < edgesWithFace.size(); ++ei)
					{
						if (wireEdge.IsSame(edgesWithFace[ei].first))
						{
							edgeInWire[ei] = true;
						}
					}
				}
			}
		}

		for (std::size_t ei = 0; ei < edgesWithFace.size(); ++ei)
		{
			if (!wires.empty() && edgeInWire[ei])
			{
				continue;
			}
			LayerSegment segment;
			if (!discretizeEdgeToSegment(edgesWithFace[ei].first, useChordHeight, uniformSegs, chordHeight,
										 layerKeepStart, layerKeepEnd, segment))
			{
				continue;
			}
			segments.push_back(std::move(segment));
		}
	}

	if (segments.empty())
	{
		for (const auto& ef : edgesWithFace)
		{
			LayerSegment segment;
			if (!discretizeEdgeToSegment(ef.first, useChordHeight, uniformSegs, chordHeight, layerKeepStart,
										 layerKeepEnd, segment))
			{
				continue;
			}
			segments.push_back(std::move(segment));
		}
	}

	std::vector<Point3d> layerPts;
	if (!stitchLayerSegments(segments, kSeamTol, zhiMode, layerPts))
	{
		return false;
	}

	if (doReverse)
	{
		std::reverse(layerPts.begin(), layerPts.end());
	}

	const std::size_t beforeCount = path.points.size();
	appendCuttingPointsToPath(layerPts, faces, path, kSeamTol);
	return path.points.size() > beforeCount;
}

bool mergeSelectedFaces(const std::vector<TopoDS_Face>& faces, TopoDS_Shape& outShape, TopoDS_Face& outRefFace)
{
	if (faces.empty())
	{
		return false;
	}
	if (faces.size() == 1)
	{
		outRefFace = faces.front();
		outShape = outRefFace;
		return true;
	}

	const double fuzzy = Precision::Confusion() * 10.0;
	TopoDS_Shape fused = faces.front();
	for (std::size_t i = 1; i < faces.size(); ++i)
	{
		BRepAlgoAPI_Fuse fuseOp(fused, faces[i]);
		fuseOp.SetFuzzyValue(fuzzy);
		fuseOp.Build();
		if (!fuseOp.IsDone())
		{
			return false;
		}
		fused = fuseOp.Shape();
	}

	ShapeUpgrade_UnifySameDomain unify(fused, Standard_True, Standard_True, Standard_True);
	unify.Build();
	fused = unify.Shape();
	if (fused.IsNull())
	{
		return false;
	}

	TopoDS_Face largestFace;
	double maxArea = -1.0;
	for (TopExp_Explorer ex(fused, TopAbs_FACE); ex.More(); ex.Next())
	{
		const TopoDS_Face candidate = TopoDS::Face(ex.Current());
		GProp_GProps props;
		BRepGProp::SurfaceProperties(candidate, props);
		const double area = props.Mass();
		if (area > maxArea)
		{
			maxArea = area;
			largestFace = candidate;
		}
	}
	if (largestFace.IsNull())
	{
		return false;
	}

	outRefFace = largestFace;
	outShape = fused;
	return true;
}

void addShapeToBox(const TopoDS_Shape& shape, Bnd_Box& box, bool& hasBox)
{
	Bnd_Box shapeBox;
	BRepBndLib::Add(shape, shapeBox);
	if (shapeBox.IsOpen())
	{
		return;
	}
	if (!hasBox)
	{
		box = shapeBox;
		hasBox = true;
	}
	else
	{
		box.Add(shapeBox);
	}
}

} // namespace

bool discretizeFaceSectionGrid(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
							   std::string* errMsg)
{
	if (input.geometry.faceIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "FaceSection requires faceIndices";
		}
		return false;
	}

	const double spacing = paramDouble(input.params, "stepMm", 2.0);
	if (spacing < 0.1)
	{
		if (errMsg)
		{
			*errMsg = "section spacing must be >= 0.1 mm";
		}
		return false;
	}

	std::vector<TopoDS_Face> faces;
	faces.reserve(input.geometry.faceIndices.size());
	for (const int faceIdx : input.geometry.faceIndices)
	{
		TopoDS_Face face;
		if (!shapeFaceAtIndex(shape, faceIdx, face, errMsg))
		{
			return false;
		}
		faces.push_back(face);
	}

	TopoDS_Shape mergedShape;
	TopoDS_Face mergedRefFace;
	const bool useMergedFace = mergeSelectedFaces(faces, mergedShape, mergedRefFace);

	Bnd_Box totalBox;
	bool hasBox = false;
	if (useMergedFace)
	{
		addShapeToBox(mergedShape, totalBox, hasBox);
	}
	else
	{
		for (const TopoDS_Face& face : faces)
		{
			addShapeToBox(face, totalBox, hasBox);
		}
	}

	const gp_Dir sectionNormal = computeSectionNormal(paramDouble(input.params, "sectionRxDeg", 0.0),
													  paramDouble(input.params, "sectionRyDeg", 0.0));
	const gp_Pnt sectionOrigin(paramDouble(input.params, "sectionOriginX", 0.0),
							   paramDouble(input.params, "sectionOriginY", 0.0),
							   paramDouble(input.params, "sectionOriginZ", 0.0));

	std::vector<double> sectionOffsets;
	if (hasBox && !totalBox.IsOpen())
	{
		double x1 = 0.0;
		double y1 = 0.0;
		double z1 = 0.0;
		double x2 = 0.0;
		double y2 = 0.0;
		double z2 = 0.0;
		totalBox.Get(x1, y1, z1, x2, y2, z2);
		double dMin = 1e30;
		double dMax = -1e30;
		for (int i = 0; i < 8; ++i)
		{
			const gp_Pnt c((i & 1) ? x2 : x1, (i & 2) ? y2 : y1, (i & 4) ? z2 : z1);
			const double d = gp_Vec(sectionOrigin, c).Dot(gp_Vec(sectionNormal));
			dMin = std::min(dMin, d);
			dMax = std::max(dMax, d);
		}
		for (double d = dMin; d <= dMax + 1e-6; d += spacing)
		{
			sectionOffsets.push_back(d);
		}
		if (sectionOffsets.empty())
		{
			sectionOffsets.push_back(0.0);
		}
	}
	else
	{
		sectionOffsets.push_back(0.0);
	}

	std::vector<std::vector<std::pair<TopoDS_Edge, TopoDS_Face>>> sectionEdgesByPlane;
	sectionEdgesByPlane.reserve(sectionOffsets.size());

	for (const double offset : sectionOffsets)
	{
		const gp_Pnt planeCenter = sectionOrigin.Translated(gp_Vec(sectionNormal) * offset);
		const gp_Pln pln(planeCenter, sectionNormal);

		std::vector<std::pair<TopoDS_Edge, TopoDS_Face>> edgesWithFace;
		if (useMergedFace)
		{
			sectionShapeWithPlane(mergedShape, mergedRefFace, pln, edgesWithFace);
		}
		else
		{
			for (const TopoDS_Face& face : faces)
			{
				sectionFaceWithPlane(face, pln, edgesWithFace);
			}
		}
		if (!edgesWithFace.empty())
		{
			sectionEdgesByPlane.push_back(std::move(edgesWithFace));
		}
	}

	if (sectionEdgesByPlane.empty())
	{
		if (errMsg)
		{
			*errMsg = "section planes produced no intersection edges";
		}
		return false;
	}

	const TrajConnectMode connectMode = parseTrajConnectMode(paramString(input.params, "trajConnectMode", "Bow"));
	const bool zhiMode = (connectMode == TrajConnectMode::Zhi);
	const bool useChordHeight = paramString(input.params, "edgeDiscretizeMode", "Uniform") == "ChordHeight";
	const int uniformSegs = std::max(2, paramInt(input.params, "uvCountU", 3));
	const double chordHeight = paramDouble(input.params, "linearDeflectionMm", 0.01);

	bool layerReverse = paramBool(input.params, "reverseLayer", false);
	for (std::size_t planeIdx = 0; planeIdx < sectionEdgesByPlane.size(); ++planeIdx)
	{
		const auto& edgesWithFace = sectionEdgesByPlane[planeIdx];
		bool doReverse = paramBool(input.params, "reverseLayer", false);
		if (connectMode == TrajConnectMode::Bow)
		{
			doReverse = layerReverse;
		}

		const std::size_t beforeCount = out.points.size();
		if (!discretizeLayer(edgesWithFace, useChordHeight, uniformSegs, chordHeight,
							 paramInt(input.params, "layerKeepStart", 0), paramInt(input.params, "layerKeepEnd", 0),
							 zhiMode, doReverse, faces, out))
		{
			continue;
		}
		if (out.points.size() > beforeCount)
		{
			out.segmentEndExclusive.push_back(out.points.size());
		}

		if (connectMode == TrajConnectMode::Bow)
		{
			layerReverse = !layerReverse;
		}
	}

	if (out.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "face section discretization produced no points";
		}
		return false;
	}
	const DiscretizeParams disc = buildDiscretizeParams(input.params);
	detail::assignPathChordTangents(out, false, disc.outputTangent, &out.segmentEndExclusive);
	return true;
}

} // namespace geoalgo
