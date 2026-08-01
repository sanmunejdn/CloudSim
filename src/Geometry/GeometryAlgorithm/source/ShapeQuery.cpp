/// @file ShapeQuery.cpp
/// @brief ShapeQuery 实现

#include "ShapeQuery.h"

#include "BrepBoolean.h"
#include "Discretize.h"
#include "Intersection.h"
#include "MeshDiscretize.h"
#include "ShapeHandle.h"
#include "ShapeIo.h"
#include "ShellOps.h"
#include "WireOps.h"
#include "detail/OccIncludes.h"

#include <BRepGProp.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GProp_GProps.hxx>

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace geoalgo
{
namespace
{
bool loadStepOrErr(const std::string& path, TopoDS_Shape& shape, std::string* errMsg)
{
	return readStepShape(path, shape, errMsg);
}

bool indexOutOfRange(std::string* errMsg, const char* what, int index, int count)
{
	if (errMsg)
	{
		*errMsg = std::string(what) + " index " + std::to_string(index) +
				  " out of range (count=" + std::to_string(count) + ")";
	}
	return false;
}

} // namespace

int shapeEdgeCount(const TopoDS_Shape& shape)
{
	int count = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next())
	{
		++count;
	}
	return count;
}

int shapeHandleEdgeCount(const ShapeHandle& handle)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(handle, &native) || native.IsNull())
		return 0;
	return shapeEdgeCount(native);
}

int shapeFaceCount(const TopoDS_Shape& shape)
{
	int count = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
	{
		++count;
	}
	return count;
}

bool shapeEdgeAtIndex(const TopoDS_Shape& shape, int index, TopoDS_Edge& outEdge, std::string* errMsg)
{
	if (index < 0)
	{
		return indexOutOfRange(errMsg, "edge", index, shapeEdgeCount(shape));
	}
	int i = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++i)
	{
		if (i == index)
		{
			outEdge = TopoDS::Edge(exp.Current());
			return true;
		}
	}
	return indexOutOfRange(errMsg, "edge", index, i);
}

bool shapeFaceAtIndex(const TopoDS_Shape& shape, int index, TopoDS_Face& outFace, std::string* errMsg)
{
	if (index < 0)
	{
		return indexOutOfRange(errMsg, "face", index, shapeFaceCount(shape));
	}
	int i = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++i)
	{
		if (i == index)
		{
			outFace = TopoDS::Face(exp.Current());
			return true;
		}
	}
	return indexOutOfRange(errMsg, "face", index, i);
}

bool discretizeStepEdgesToPolylines(const std::string& pathLocal, const TessellateParams& params,
									std::vector<Polyline3d>& outPolylines, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	return discretizeShapeEdges(shape, params, outPolylines, errMsg);
}

bool discretizeStepFaceToMesh(const std::string& pathLocal, int faceIndex, const MeshDiscretizeParams& params,
							  std::vector<float>& soup, MeshDiscretizeReport& report, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	TopoDS_Face face;
	if (!shapeFaceAtIndex(shape, faceIndex, face, errMsg))
	{
		return false;
	}
	if (!discretizeFaceToMesh(face, params, soup, errMsg))
	{
		return false;
	}
	fillMeshReport(soup, report);
	report.modeUsed = params.mode;
	return true;
}

bool intersectStepEdges(const std::string& pathLocal, int edgeIndex1, int edgeIndex2, const IntersectionParams& params,
						IntersectionResult& result, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	TopoDS_Edge e1;
	TopoDS_Edge e2;
	if (!shapeEdgeAtIndex(shape, edgeIndex1, e1, errMsg) || !shapeEdgeAtIndex(shape, edgeIndex2, e2, errMsg))
	{
		return false;
	}
	return intersectEdges(e1, e2, params, result, errMsg);
}

bool intersectStepEdgeFace(const std::string& pathLocal, int edgeIndex, int faceIndex, const IntersectionParams& params,
						   IntersectionResult& result, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	TopoDS_Edge edge;
	TopoDS_Face face;
	if (!shapeEdgeAtIndex(shape, edgeIndex, edge, errMsg) || !shapeFaceAtIndex(shape, faceIndex, face, errMsg))
	{
		return false;
	}
	return intersectEdgeFace(edge, face, params, result, errMsg);
}

bool intersectStepFaces(const std::string& pathLocal, int faceIndex1, int faceIndex2, const IntersectionParams& params,
						IntersectionResult& result, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	TopoDS_Face f1;
	TopoDS_Face f2;
	if (!shapeFaceAtIndex(shape, faceIndex1, f1, errMsg) || !shapeFaceAtIndex(shape, faceIndex2, f2, errMsg))
	{
		return false;
	}
	return intersectFaces(f1, f2, params, result, errMsg);
}

bool intersectStepFiles(const std::string& pathLocal1, const std::string& pathLocal2, const IntersectionParams& params,
						IntersectionResult& result, std::string* errMsg)
{
	TopoDS_Shape s1;
	TopoDS_Shape s2;
	if (!loadStepOrErr(pathLocal1, s1, errMsg) || !loadStepOrErr(pathLocal2, s2, errMsg))
	{
		return false;
	}
	return intersectShapes(s1, s2, params, result, errMsg);
}

bool brepBooleanStepFilesToMesh(const std::string& targetPathLocal, const std::string& toolPathLocal, BrepBooleanOp op,
								const MeshDiscretizeParams& meshParams, std::vector<float>& outSoup,
								std::string* errMsg)
{
	TopoDS_Shape target;
	TopoDS_Shape tool;
	if (!loadStepOrErr(targetPathLocal, target, errMsg) || !loadStepOrErr(toolPathLocal, tool, errMsg))
	{
		return false;
	}
	return brepBooleanToMesh(target, tool, op, meshParams, outSoup, errMsg);
}

bool fuseStepEdgesToPolyline(const std::string& pathLocal, const std::vector<int>& edgeIndices,
							 const TessellateParams& disc, Polyline3d& out, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	std::vector<TopoDS_Wire> wires;
	wires.reserve(edgeIndices.size());
	for (int idx : edgeIndices)
	{
		TopoDS_Edge edge;
		if (!shapeEdgeAtIndex(shape, idx, edge, errMsg))
		{
			return false;
		}
		BRepBuilderAPI_MakeWire mk(edge);
		if (!mk.IsDone())
		{
			if (errMsg)
			{
				*errMsg = "failed to build wire from edge";
			}
			return false;
		}
		wires.push_back(mk.Wire());
	}
	return fuseWiresToPolyline(wires, disc, out, errMsg);
}

bool sewStepFacesToMesh(const std::string& pathLocal, const std::vector<int>& faceIndices, double toleranceMm,
						const MeshDiscretizeParams& meshParams, std::vector<float>& outSoup, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	std::vector<TopoDS_Face> faces;
	faces.reserve(faceIndices.size());
	for (int idx : faceIndices)
	{
		TopoDS_Face face;
		if (!shapeFaceAtIndex(shape, idx, face, errMsg))
		{
			return false;
		}
		faces.push_back(face);
	}
	return sewFacesToMesh(faces, toleranceMm, meshParams, outSoup, errMsg);
}

namespace
{
gp_Pnt toGpPnt(const Point3d& p)
{
	return gp_Pnt(p.x, p.y, p.z);
}

gp_Dir toGpDir(const Point3d& d)
{
	const double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
	if (len < 1e-12)
	{
		return gp_Dir(0.0, 0.0, 1.0);
	}
	return gp_Dir(d.x / len, d.y / len, d.z / len);
}

int faceIndexOfFace(const TopoDS_Shape& shape, const TopoDS_Face& target)
{
	int idx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++idx)
	{
		if (TopoDS::Face(exp.Current()).IsSame(target))
		{
			return idx;
		}
	}
	return -1;
}

int edgeIndexOfEdge(const TopoDS_Shape& shape, const TopoDS_Edge& target)
{
	int idx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++idx)
	{
		if (TopoDS::Edge(exp.Current()).IsSame(target))
		{
			return idx;
		}
	}
	return -1;
}

bool nativeShapeOrErr(const ShapeHandle& handle, TopoDS_Shape& out, std::string* errMsg)
{
	if (handle.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	if (!ShapeHandleAccess::nativeShape(handle, &out))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	return true;
}

double edgeDistanceToPointMm(const TopoDS_Edge& edge, const gp_Pnt& query, gp_Pnt& outClosest)
{
	BRepAdaptor_Curve curve(edge);
	const Handle(Geom_Curve) geom = curve.Curve().Curve();
	if (geom.IsNull())
	{
		return (std::numeric_limits<double>::max)();
	}
	GeomAPI_ProjectPointOnCurve proj(query, geom);
	if (proj.NbPoints() < 1)
	{
		return (std::numeric_limits<double>::max)();
	}
	outClosest = proj.NearestPoint();
	return proj.LowerDistance();
}

bool shapeRayParameterRange(const TopoDS_Shape& shape, const gp_Lin& line, Standard_Real& tInf, Standard_Real& tSup)
{
	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	if (box.IsVoid())
	{
		tInf = -1.0e6;
		tSup = 1.0e6;
		return false;
	}
	Standard_Real xmin = 0.0;
	Standard_Real ymin = 0.0;
	Standard_Real zmin = 0.0;
	Standard_Real xmax = 0.0;
	Standard_Real ymax = 0.0;
	Standard_Real zmax = 0.0;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	const gp_Pnt corners[8] = {
		gp_Pnt(xmin, ymin, zmin), gp_Pnt(xmax, ymin, zmin), gp_Pnt(xmin, ymax, zmin), gp_Pnt(xmax, ymax, zmin),
		gp_Pnt(xmin, ymin, zmax), gp_Pnt(xmax, ymin, zmax), gp_Pnt(xmin, ymax, zmax), gp_Pnt(xmax, ymax, zmax),
	};
	tInf = (std::numeric_limits<Standard_Real>::max)();
	tSup = -(std::numeric_limits<Standard_Real>::max)();
	for (const gp_Pnt& corner : corners)
	{
		const Standard_Real t = ElCLib::Parameter(line, corner);
		if (t < tInf)
		{
			tInf = t;
		}
		if (t > tSup)
		{
			tSup = t;
		}
	}
	const Standard_Real span = tSup - tInf;
	const Standard_Real margin = (span > 1e-6) ? span * 0.05 : 100.0;
	tInf -= margin;
	tSup += margin;
	return true;
}

bool pickClosestEdgeInExplorer(const TopoDS_Shape& shape, const TopExp_Explorer& edgeExpStart, const gp_Pnt& query,
							   const double toleranceMm, int& outEdgeIdx, gp_Pnt& outClosest)
{
	TopTools_IndexedMapOfShape edgeMap;
	TopExp::MapShapes(shape, TopAbs_EDGE, edgeMap);
	double bestDist = (std::numeric_limits<double>::max)();
	int bestIdx = -1;
	gp_Pnt bestPt;
	for (TopExp_Explorer exp = edgeExpStart; exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		const int globalIdx = edgeMap.FindIndex(edge) - 1;
		if (globalIdx < 0)
		{
			continue;
		}
		gp_Pnt closest;
		const double dist = edgeDistanceToPointMm(edge, query, closest);
		if (dist < bestDist)
		{
			bestDist = dist;
			bestIdx = globalIdx;
			bestPt = closest;
		}
	}
	if (bestIdx < 0 || bestDist > toleranceMm)
	{
		return false;
	}
	outEdgeIdx = bestIdx;
	outClosest = bestPt;
	return true;
}

} // namespace

bool resolveFaceIndexFromModelPoint(const ShapeHandle& shapeHandle, const Point3d& modelPointMm, int& outFaceIndex,
									const double toleranceMm, std::string* errMsg)
{
	outFaceIndex = -1;
	if (shapeHandle.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	const gp_Pnt query = toGpPnt(modelPointMm);
	double bestDist = (std::numeric_limits<double>::max)();
	int bestIdx = -1;
	int faceIdx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx)
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
		if (surf.IsNull())
		{
			continue;
		}
		GeomAPI_ProjectPointOnSurf proj(query, surf);
		if (proj.NbPoints() < 1)
		{
			continue;
		}
		const double dist = proj.LowerDistance();
		if (dist < bestDist)
		{
			bestDist = dist;
			bestIdx = faceIdx;
		}
	}
	if (bestIdx < 0 || bestDist > toleranceMm)
	{
		if (errMsg)
		{
			*errMsg = "no STEP face within tolerance";
		}
		return false;
	}
	outFaceIndex = bestIdx;
	return true;
}

bool resolveStepFaceIndexFromModelPoint(const std::string& stepPathUtf8, const Point3d& modelPointMm, int& outFaceIndex,
										const double toleranceMm, std::string* errMsg)
{
	ShapeHandle handle;
	if (!readStepIntoHandle(stepPathUtf8, handle, errMsg))
	{
		return false;
	}
	return resolveFaceIndexFromModelPoint(handle, modelPointMm, outFaceIndex, toleranceMm, errMsg);
}

bool resolveEdgeIndexFromModelPoints(const ShapeHandle& shapeHandle, const Point3d& modelPointA,
									 const Point3d& modelPointB, int& outEdgeIndex, const double toleranceMm,
									 std::string* errMsg)
{
	outEdgeIndex = -1;
	if (shapeHandle.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	const gp_Pnt query(0.5 * (modelPointA.x + modelPointB.x), 0.5 * (modelPointA.y + modelPointB.y),
					   0.5 * (modelPointA.z + modelPointB.z));
	double bestDist = (std::numeric_limits<double>::max)();
	int bestIdx = -1;
	int edgeIdx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++edgeIdx)
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		BRepAdaptor_Curve curve(edge);
		GeomAPI_ProjectPointOnCurve proj(query, curve.Curve().Curve());
		if (proj.NbPoints() < 1)
		{
			continue;
		}
		const double dist = proj.LowerDistance();
		if (dist < bestDist)
		{
			bestDist = dist;
			bestIdx = edgeIdx;
		}
	}
	if (bestIdx < 0 || bestDist > toleranceMm)
	{
		if (errMsg)
		{
			*errMsg = "no STEP edge within tolerance";
		}
		return false;
	}
	outEdgeIndex = bestIdx;
	return true;
}

bool resolveStepEdgeIndexFromModelPoints(const std::string& stepPathUtf8, const Point3d& modelPointA,
										 const Point3d& modelPointB, int& outEdgeIndex, const double toleranceMm,
										 std::string* errMsg)
{
	ShapeHandle handle;
	if (!readStepIntoHandle(stepPathUtf8, handle, errMsg))
	{
		return false;
	}
	return resolveEdgeIndexFromModelPoints(handle, modelPointA, modelPointB, outEdgeIndex, toleranceMm, errMsg);
}

bool pickShapeFaceByModelRay(const ShapeHandle& shapeHandle, const Point3d& rayOriginMm, const Point3d& rayDirUnit,
							 ShapeRayPickResult& out, std::string* errMsg)
{
	out = ShapeRayPickResult{};
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
	{
		return false;
	}
	const gp_Pnt origin = toGpPnt(rayOriginMm);
	const gp_Lin line(origin, toGpDir(rayDirUnit));
	Standard_Real tInf = 0.0;
	Standard_Real tSup = 0.0;
	shapeRayParameterRange(shape, line, tInf, tSup);
	IntCurvesFace_ShapeIntersector intersector;
	intersector.Load(shape, 1e-7);
	intersector.Perform(line, tInf, tSup);
	if (!intersector.IsDone() || intersector.NbPnt() < 1)
	{
		if (errMsg)
		{
			*errMsg = "ray does not hit B-rep face";
		}
		return false;
	}
	Standard_Integer bestIdx = 1;
	Standard_Real bestW = intersector.WParameter(1);
	for (Standard_Integer i = 2; i <= intersector.NbPnt(); ++i)
	{
		const Standard_Real w = intersector.WParameter(i);
		if (w < bestW)
		{
			bestW = w;
			bestIdx = i;
		}
	}
	const TopoDS_Face face = intersector.Face(bestIdx);
	const gp_Pnt hit = intersector.Pnt(bestIdx);
	const int faceIdx = faceIndexOfFace(shape, face);
	if (faceIdx < 0)
	{
		if (errMsg)
		{
			*errMsg = "picked face index unresolved";
		}
		return false;
	}
	out.hit = true;
	out.faceIndex = faceIdx;
	out.hitPointModelMm = {hit.X(), hit.Y(), hit.Z()};
	return true;
}

bool pickShapeEdgeByModelPoint(const ShapeHandle& shapeHandle, const Point3d& queryPointModelMm,
							   const double toleranceMm, ShapeRayPickResult& out, std::string* errMsg)
{
	out = ShapeRayPickResult{};
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
	{
		return false;
	}

	const gp_Pnt query = toGpPnt(queryPointModelMm);
	int faceIdx = -1;
	const bool haveFace = resolveFaceIndexFromModelPoint(shapeHandle, queryPointModelMm, faceIdx, toleranceMm, nullptr);

	int edgeIdx = -1;
	gp_Pnt closest;
	if (haveFace && faceIdx >= 0)
	{
		TopoDS_Face face;
		if (shapeFaceAtIndex(shape, faceIdx, face, errMsg))
		{
			if (pickClosestEdgeInExplorer(shape, TopExp_Explorer(face, TopAbs_EDGE), query, toleranceMm, edgeIdx,
										  closest))
			{
				out.hit = true;
				out.faceIndex = faceIdx;
				out.edgeIndex = edgeIdx;
				out.hitPointModelMm = queryPointModelMm;
				out.edgePointModelMm = {closest.X(), closest.Y(), closest.Z()};
				return true;
			}
		}
	}

	if (pickClosestEdgeInExplorer(shape, TopExp_Explorer(shape, TopAbs_EDGE), query, toleranceMm, edgeIdx, closest))
	{
		out.hit = true;
		out.edgeIndex = edgeIdx;
		out.hitPointModelMm = queryPointModelMm;
		out.edgePointModelMm = {closest.X(), closest.Y(), closest.Z()};
		if (haveFace)
		{
			out.faceIndex = faceIdx;
		}
		return true;
	}

	if (errMsg)
	{
		*errMsg = "no B-rep edge within tolerance";
	}
	return false;
}

bool pickShapeEdgeByModelRay(const ShapeHandle& shapeHandle, const Point3d& rayOriginMm, const Point3d& rayDirUnit,
							 const double toleranceMm, ShapeRayPickResult& out, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
	{
		return false;
	}

	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	Point3d queryPoint = rayOriginMm;
	if (!box.IsVoid())
	{
		Standard_Real xmin = 0.0;
		Standard_Real ymin = 0.0;
		Standard_Real zmin = 0.0;
		Standard_Real xmax = 0.0;
		Standard_Real ymax = 0.0;
		Standard_Real zmax = 0.0;
		box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		const gp_Pnt center((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
		const gp_Pnt origin = toGpPnt(rayOriginMm);
		const gp_Dir dir = toGpDir(rayDirUnit);
		const gp_Vec offset(origin, center);
		const Standard_Real t = offset.Dot(gp_Vec(dir));
		queryPoint = {
			rayOriginMm.x + t * dir.X(),
			rayOriginMm.y + t * dir.Y(),
			rayOriginMm.z + t * dir.Z(),
		};
	}

	return pickShapeEdgeByModelPoint(shapeHandle, queryPoint, toleranceMm, out, errMsg);
}

bool validateShapeFaceIndex(const ShapeHandle& shapeHandle, const int faceIndex, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
	{
		return false;
	}
	TopoDS_Face face;
	return shapeFaceAtIndex(shape, faceIndex, face, errMsg);
}

bool validateShapeEdgeIndex(const ShapeHandle& shapeHandle, const int edgeIndex, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
	{
		return false;
	}
	TopoDS_Edge edge;
	return shapeEdgeAtIndex(shape, edgeIndex, edge, errMsg);
}

bool discretizeShapeFaceEdgesToPolylines(const ShapeHandle& shapeHandle, const int faceIndex,
										 const TessellateParams& params, std::vector<Polyline3d>& outPolylines,
										 std::string* errMsg)
{
	outPolylines.clear();
	std::vector<std::vector<int>> faceEdges;
	if (!collectShapeFaceEdgeIndices(shapeHandle, faceEdges, errMsg))
		return false;
	if (faceIndex < 0 || static_cast<std::size_t>(faceIndex) >= faceEdges.size())
	{
		if (errMsg)
			*errMsg = "face index out of range";
		return false;
	}
	const std::vector<int>& edges = faceEdges[static_cast<std::size_t>(faceIndex)];
	std::vector<int> unique;
	unique.reserve(edges.size());
	for (int ei : edges)
	{
		if (std::find(unique.begin(), unique.end(), ei) == unique.end())
			unique.push_back(ei);
	}
	outPolylines.reserve(unique.size());
	for (int ei : unique)
	{
		Polyline3d poly;
		if (!discretizeShapeEdgeByIndex(shapeHandle, ei, params, poly, errMsg))
			return false;
		if (poly.xyz.size() >= 6)
			outPolylines.push_back(std::move(poly));
	}
	if (outPolylines.empty())
	{
		if (errMsg)
			*errMsg = "face has no discretizable boundary edges";
		return false;
	}
	return true;
}

bool collectShapeFaceEdgeIndices(const ShapeHandle& shapeHandle, std::vector<std::vector<int>>& outFaceEdgeIndices,
								 std::string* errMsg)
{
	outFaceEdgeIndices.clear();
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
	{
		return false;
	}
	const int faceCount = shapeFaceCount(shape);
	if (faceCount <= 0)
	{
		if (errMsg)
		{
			*errMsg = "shape has no faces";
		}
		return false;
	}
	TopTools_IndexedMapOfShape edgeMap;
	TopExp::MapShapes(shape, TopAbs_EDGE, edgeMap);
	outFaceEdgeIndices.resize(static_cast<std::size_t>(faceCount));
	for (int faceIdx = 0; faceIdx < faceCount; ++faceIdx)
	{
		TopoDS_Face face;
		if (!shapeFaceAtIndex(shape, faceIdx, face, errMsg))
		{
			return false;
		}
		std::vector<int>& edges = outFaceEdgeIndices[static_cast<std::size_t>(faceIdx)];
		for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next())
		{
			const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
			const int edgeIdx = edgeMap.FindIndex(edge) - 1;
			if (edgeIdx >= 0)
			{
				edges.push_back(edgeIdx);
			}
		}
	}
	return true;
}

void mergeFaceOwnershipByTShape(const ShapeHandle& tip, const std::string& featureId,
								std::unordered_map<std::uintptr_t, std::string>& tshapeOwners,
								std::unordered_map<int, std::string>& outFaceIndexOwners)
{
	outFaceIndexOwners.clear();
	if (tip.isNull() || featureId.empty())
		return;
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(tip, &native) || native.IsNull())
		return;
	int idx = 0;
	for (TopExp_Explorer ex(native, TopAbs_FACE); ex.More(); ex.Next(), ++idx)
	{
		const TopoDS_Face f = TopoDS::Face(ex.Current());
		const auto key = reinterpret_cast<std::uintptr_t>(f.TShape().get());
		if (key == 0)
			continue;
		if (tshapeOwners.find(key) == tshapeOwners.end())
			tshapeOwners.emplace(key, featureId);
		const auto it = tshapeOwners.find(key);
		if (it != tshapeOwners.end())
			outFaceIndexOwners[idx] = it->second;
	}
}

namespace
{
void appendPnt(std::vector<float>& xyz, const gp_Pnt& p)
{
	xyz.push_back(static_cast<float>(p.X()));
	xyz.push_back(static_cast<float>(p.Y()));
	xyz.push_back(static_cast<float>(p.Z()));
}

double edgeLengthMmOf(const TopoDS_Edge& edge)
{
	GProp_GProps props;
	BRepGProp::LinearProperties(edge, props);
	return props.Mass();
}

bool isFullCircleSpan(const BRepAdaptor_Curve& curve, const TopoDS_Edge& edge)
{
	const double first = curve.FirstParameter();
	const double last = curve.LastParameter();
	const double span = std::abs(last - first);
	// 周期圆容差略放宽，避免近满圆被当成 Arc 再取错误 mid
	return edge.Closed() || std::abs(span - 2.0 * M_PI) < 1e-2 || span > 2.0 * M_PI - 1e-2;
}

double arcMidParameter(const BRepAdaptor_Curve& curve)
{
	const double first = curve.FirstParameter();
	const double last = curve.LastParameter();
	if (last >= first - 1e-12)
		return 0.5 * (first + last);
	if (curve.IsPeriodic())
	{
		const double period = curve.Period();
		double span = last - first;
		if (span < 0.0)
			span += period;
		return first + 0.5 * span;
	}
	return 0.5 * (first + last);
}
} // namespace

bool extractShapeFaceBoundarySegments(const ShapeHandle& shapeHandle, const int faceIndex,
									  const TessellateParams& fallbackTess, std::vector<FaceBoundarySeg>& outSegs,
									  std::string* errMsg)
{
	outSegs.clear();
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
		return false;
	TopoDS_Face face;
	if (!shapeFaceAtIndex(shape, faceIndex, face, errMsg))
		return false;

	TopTools_IndexedMapOfShape seenEdges;
	for (TopExp_Explorer wireExp(face, TopAbs_WIRE); wireExp.More(); wireExp.Next())
	{
		const TopoDS_Wire wire = TopoDS::Wire(wireExp.Current());
		for (BRepTools_WireExplorer edgeExp(wire, face); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge edge = edgeExp.Current();
			if (seenEdges.Contains(edge))
				continue;
			seenEdges.Add(edge);

			BRepAdaptor_Curve curve(edge);
			FaceBoundarySeg seg;
			if (curve.GetType() == GeomAbs_Line)
			{
				seg.kind = FaceBoundarySegKind::Line;
				appendPnt(seg.xyz, curve.Value(curve.FirstParameter()));
				appendPnt(seg.xyz, curve.Value(curve.LastParameter()));
				outSegs.push_back(std::move(seg));
				continue;
			}
			if (curve.GetType() == GeomAbs_Circle)
			{
				const gp_Circ circ = curve.Circle();
				const double radius = circ.Radius();
				if (radius > 1e-9 && isFullCircleSpan(curve, edge))
				{
					seg.kind = FaceBoundarySegKind::Circle;
					seg.radiusMm = radius;
					appendPnt(seg.xyz, circ.Location());
					appendPnt(seg.xyz, curve.Value(curve.FirstParameter()));
					outSegs.push_back(std::move(seg));
					continue;
				}
				if (radius > 1e-9)
				{
					seg.kind = FaceBoundarySegKind::Arc;
					const double midParam = arcMidParameter(curve);
					appendPnt(seg.xyz, curve.Value(curve.FirstParameter()));
					appendPnt(seg.xyz, curve.Value(midParam));
					appendPnt(seg.xyz, curve.Value(curve.LastParameter()));
					outSegs.push_back(std::move(seg));
					continue;
				}
			}

			Polyline3d poly;
			const int ei = edgeIndexOfEdge(shape, edge);
			if (ei < 0)
			{
				BRepAdaptor_Curve c2(edge);
				poly.xyz.clear();
				constexpr int segs = 24;
				for (int i = 0; i <= segs; ++i)
				{
					const double t =
						c2.FirstParameter() +
						(c2.LastParameter() - c2.FirstParameter()) * (static_cast<double>(i) / segs);
					appendPnt(poly.xyz, c2.Value(t));
				}
			}
			else if (!discretizeShapeEdgeByIndex(shapeHandle, ei, fallbackTess, poly, errMsg))
				return false;
			if (poly.xyz.size() >= 6)
			{
				seg.kind = FaceBoundarySegKind::Polyline;
				seg.xyz = std::move(poly.xyz);
				outSegs.push_back(std::move(seg));
			}
		}
	}

	if (outSegs.empty())
	{
		if (errMsg)
			*errMsg = "face has no boundary segments";
		return false;
	}
	return true;
}

bool selectLongestEdgeIndices(const ShapeHandle& shapeHandle, int topK, std::vector<int>& outEdgeIndices,
							  std::string* errMsg)
{
	outEdgeIndices.clear();
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
		return false;
	std::vector<std::pair<double, int>> scored;
	int idx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++idx)
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		scored.emplace_back(edgeLengthMmOf(edge), idx);
	}
	if (scored.empty())
	{
		if (errMsg)
			*errMsg = "shape has no edges";
		return false;
	}
	std::sort(scored.begin(), scored.end(),
			  [](const auto& a, const auto& b) { return a.first > b.first; });
	const int k = topK <= 0 ? 4 : topK;
	const int n = static_cast<int>(std::min(scored.size(), static_cast<size_t>(k)));
	outEdgeIndices.reserve(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
		outEdgeIndices.push_back(scored[static_cast<size_t>(i)].second);
	return true;
}

bool selectTopBoundaryEdgeIndices(const ShapeHandle& shapeHandle, std::vector<int>& outEdgeIndices,
								  std::string* errMsg)
{
	outEdgeIndices.clear();
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
		return false;

	int bestFace = -1;
	double bestZ = -(std::numeric_limits<double>::max)();
	int faceIdx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx)
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		BRepAdaptor_Surface surf(face);
		if (surf.GetType() != GeomAbs_Plane)
			continue;
		gp_Pln pln = surf.Plane();
		gp_Dir n = pln.Axis().Direction();
		if (face.Orientation() == TopAbs_REVERSED)
			n.Reverse();
		// 近似「顶面」：法向朝 +Z 且原点 Z 最大
		if (n.Z() < 0.5)
			continue;
		const double z = pln.Location().Z();
		if (z > bestZ)
		{
			bestZ = z;
			bestFace = faceIdx;
		}
	}

	if (bestFace < 0)
		return selectLongestEdgeIndices(shapeHandle, 4, outEdgeIndices, errMsg);

	std::vector<std::vector<int>> faceEdges;
	if (!collectShapeFaceEdgeIndices(shapeHandle, faceEdges, errMsg))
		return false;
	if (static_cast<size_t>(bestFace) >= faceEdges.size())
	{
		if (errMsg)
			*errMsg = "top face index out of range";
		return false;
	}
	outEdgeIndices = faceEdges[static_cast<size_t>(bestFace)];
	std::sort(outEdgeIndices.begin(), outEdgeIndices.end());
	outEdgeIndices.erase(std::unique(outEdgeIndices.begin(), outEdgeIndices.end()), outEdgeIndices.end());
	if (outEdgeIndices.empty())
	{
		if (errMsg)
			*errMsg = "top face has no edges";
		return false;
	}
	return true;
}

int shapeHandleVertexCount(const ShapeHandle& handle)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(handle, &native) || native.IsNull())
		return 0;
	TopTools_IndexedMapOfShape map;
	TopExp::MapShapes(native, TopAbs_VERTEX, map);
	return map.Extent();
}

bool shapeVertexPointAtIndex(const ShapeHandle& handle, int vertexIndex, Point3d& outPointMm, std::string* errMsg)
{
	TopoDS_Shape native;
	if (!nativeShapeOrErr(handle, native, errMsg))
		return false;
	TopTools_IndexedMapOfShape map;
	TopExp::MapShapes(native, TopAbs_VERTEX, map);
	if (vertexIndex < 0 || vertexIndex >= map.Extent())
	{
		if (errMsg)
			*errMsg = "vertex index out of range";
		return false;
	}
	const TopoDS_Vertex v = TopoDS::Vertex(map.FindKey(vertexIndex + 1));
	const gp_Pnt p = BRep_Tool::Pnt(v);
	outPointMm = {p.X(), p.Y(), p.Z()};
	return true;
}

bool pickShapeVertexByModelPoint(const ShapeHandle& shapeHandle, const Point3d& queryPointModelMm,
								 const double toleranceMm, int& outVertexIndex, Point3d& outVertexModelMm,
								 std::string* errMsg)
{
	outVertexIndex = -1;
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(shapeHandle, shape, errMsg))
		return false;
	TopTools_IndexedMapOfShape map;
	TopExp::MapShapes(shape, TopAbs_VERTEX, map);
	const gp_Pnt query = toGpPnt(queryPointModelMm);
	double bestDist2 = (std::numeric_limits<double>::max)();
	int bestIdx = -1;
	gp_Pnt bestPt;
	for (int i = 1; i <= map.Extent(); ++i)
	{
		const TopoDS_Vertex v = TopoDS::Vertex(map.FindKey(i));
		const gp_Pnt p = BRep_Tool::Pnt(v);
		const double dx = p.X() - query.X();
		const double dy = p.Y() - query.Y();
		const double dz = p.Z() - query.Z();
		const double d2 = dx * dx + dy * dy + dz * dz;
		if (d2 < bestDist2)
		{
			bestDist2 = d2;
			bestIdx = i - 1;
			bestPt = p;
		}
	}
	if (bestIdx < 0 || bestDist2 > toleranceMm * toleranceMm)
	{
		if (errMsg)
			*errMsg = "no B-rep vertex within tolerance";
		return false;
	}
	outVertexIndex = bestIdx;
	outVertexModelMm = {bestPt.X(), bestPt.Y(), bestPt.Z()};
	return true;
}

bool shapeHandleEdgeEndpoints(const ShapeHandle& handle, int edgeIndex, Point3d& outAMm, Point3d& outBMm,
							  std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!nativeShapeOrErr(handle, shape, errMsg))
		return false;
	TopoDS_Edge edge;
	if (!shapeEdgeAtIndex(shape, edgeIndex, edge, errMsg))
		return false;
	TopExp_Explorer ex(edge, TopAbs_VERTEX);
	if (!ex.More())
	{
		if (errMsg)
			*errMsg = "edge has no vertices";
		return false;
	}
	const gp_Pnt a = BRep_Tool::Pnt(TopoDS::Vertex(ex.Current()));
	ex.Next();
	if (!ex.More())
	{
		if (errMsg)
			*errMsg = "edge has only one vertex";
		return false;
	}
	const gp_Pnt b = BRep_Tool::Pnt(TopoDS::Vertex(ex.Current()));
	outAMm = {a.X(), a.Y(), a.Z()};
	outBMm = {b.X(), b.Y(), b.Z()};
	return true;
}

} // namespace geoalgo
