#include "detail/OccIncludes.h"

#include "BrepBoolean.h"
#include "Discretize.h"
#include "Intersection.h"
#include "MeshDiscretize.h"
#include "ShapeIo.h"
#include "ShapeQuery.h"
#include "ShellOps.h"
#include "WireOps.h"

#include <cmath>
#include <limits>

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
		*errMsg = std::string(what) + " index " + std::to_string(index) + " out of range (count=" + std::to_string(count) + ")";
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

bool discretizeStepEdgesToPolylines(
	const std::string& pathLocal,
	const TessellateParams& params,
	std::vector<Polyline3d>& outPolylines,
	std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!loadStepOrErr(pathLocal, shape, errMsg))
	{
		return false;
	}
	return discretizeShapeEdges(shape, params, outPolylines, errMsg);
}

bool discretizeStepFaceToMesh(
	const std::string& pathLocal,
	int faceIndex,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	MeshDiscretizeReport& report,
	std::string* errMsg)
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

bool intersectStepEdges(
	const std::string& pathLocal,
	int edgeIndex1,
	int edgeIndex2,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg)
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

bool intersectStepEdgeFace(
	const std::string& pathLocal,
	int edgeIndex,
	int faceIndex,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg)
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

bool intersectStepFaces(
	const std::string& pathLocal,
	int faceIndex1,
	int faceIndex2,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg)
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

bool intersectStepFiles(
	const std::string& pathLocal1,
	const std::string& pathLocal2,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg)
{
	TopoDS_Shape s1;
	TopoDS_Shape s2;
	if (!loadStepOrErr(pathLocal1, s1, errMsg) || !loadStepOrErr(pathLocal2, s2, errMsg))
	{
		return false;
	}
	return intersectShapes(s1, s2, params, result, errMsg);
}

bool brepBooleanStepFilesToMesh(
	const std::string& targetPathLocal,
	const std::string& toolPathLocal,
	BrepBooleanOp op,
	const MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
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

bool fuseStepEdgesToPolyline(
	const std::string& pathLocal,
	const std::vector<int>& edgeIndices,
	const TessellateParams& disc,
	Polyline3d& out,
	std::string* errMsg)
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

bool sewStepFacesToMesh(
	const std::string& pathLocal,
	const std::vector<int>& faceIndices,
	double toleranceMm,
	const MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg)
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

} // namespace

bool resolveStepFaceIndexFromModelPoint(
	const std::string& stepPathUtf8,
	const Point3d& modelPointMm,
	int& outFaceIndex,
	const double toleranceMm,
	std::string* errMsg)
{
	outFaceIndex = -1;
	TopoDS_Shape shape;
	if (!loadStepOrErr(stepPathUtf8, shape, errMsg))
	{
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

bool resolveStepEdgeIndexFromModelPoints(
	const std::string& stepPathUtf8,
	const Point3d& modelPointA,
	const Point3d& modelPointB,
	int& outEdgeIndex,
	const double toleranceMm,
	std::string* errMsg)
{
	outEdgeIndex = -1;
	TopoDS_Shape shape;
	if (!loadStepOrErr(stepPathUtf8, shape, errMsg))
	{
		return false;
	}
	const gp_Pnt query(
		0.5 * (modelPointA.x + modelPointB.x),
		0.5 * (modelPointA.y + modelPointB.y),
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

} // namespace geoalgo
