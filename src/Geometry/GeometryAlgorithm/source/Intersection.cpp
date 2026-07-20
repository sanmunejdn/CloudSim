/// @file Intersection.cpp
/// @brief Intersection 实现

#include "Intersection.h"

#include "Discretize.h"
#include "detail/OccIncludes.h"

#include <cmath>

namespace geoalgo
{
namespace
{
void appendHit(IntersectionResult& result, const gp_Pnt& p)
{
	IntersectionHit hit;
	hit.positionMm.x = p.X();
	hit.positionMm.y = p.Y();
	hit.positionMm.z = p.Z();
	result.points.push_back(hit);
}

bool wireToPolylines(const TopoDS_Shape& section, const IntersectionParams& params, IntersectionResult& result,
					 std::string* errMsg)
{
	for (TopExp_Explorer exp(section, TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		if (params.discretizeCurves)
		{
			Polyline3d poly;
			if (!discretizeEdge(edge, params.curveDisc, poly, errMsg))
			{
				return false;
			}
			if (!poly.xyz.empty())
			{
				result.curves.push_back(std::move(poly));
			}
		}
		else
		{
			const BRepAdaptor_Curve curve(edge);
			appendHit(result, curve.Value(curve.FirstParameter()));
			appendHit(result, curve.Value(curve.LastParameter()));
		}
	}
	return true;
}

} // namespace

bool intersectEdges(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, const IntersectionParams& params,
					IntersectionResult& result, std::string* errMsg)
{
	result = {};
	BRepAlgoAPI_Section section(edge1, edge2, Standard_False);
	section.Approximation(Standard_True);
	section.Build();
	if (!section.IsDone())
	{
		if (errMsg)
		{
			*errMsg = "edge-edge intersection failed";
		}
		return false;
	}
	return wireToPolylines(section.Shape(), params, result, errMsg);
}

bool intersectEdgeFace(const TopoDS_Edge& edge, const TopoDS_Face& face, const IntersectionParams& params,
					   IntersectionResult& result, std::string* errMsg)
{
	result = {};
	BRepAlgoAPI_Section section(edge, face, Standard_False);
	section.ComputePCurveOn1(Standard_False);
	section.Approximation(Standard_True);
	section.Build();
	if (!section.IsDone())
	{
		if (errMsg)
		{
			*errMsg = "BRepAlgoAPI_Section edge-face failed";
		}
		return false;
	}
	return wireToPolylines(section.Shape(), params, result, errMsg);
}

bool intersectFaces(const TopoDS_Face& face1, const TopoDS_Face& face2, const IntersectionParams& params,
					IntersectionResult& result, std::string* errMsg)
{
	result = {};
	BRepAlgoAPI_Section section(face1, face2, Standard_False);
	section.Approximation(Standard_True);
	section.Build();
	if (!section.IsDone())
	{
		if (errMsg)
		{
			*errMsg = "face-face intersection failed";
		}
		return false;
	}
	return wireToPolylines(section.Shape(), params, result, errMsg);
}

bool intersectShapes(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2, const IntersectionParams& params,
					 IntersectionResult& result, std::string* errMsg)
{
	result = {};
	BRepAlgoAPI_Section section(shape1, shape2, Standard_False);
	section.Approximation(Standard_True);
	section.Build();
	if (!section.IsDone())
	{
		if (errMsg)
		{
			*errMsg = "BRepAlgoAPI_Section failed";
		}
		return false;
	}
	return wireToPolylines(section.Shape(), params, result, errMsg);
}

} // namespace geoalgo
