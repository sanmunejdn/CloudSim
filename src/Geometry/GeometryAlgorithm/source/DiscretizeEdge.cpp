#include "detail/OccIncludes.h"

#include "Discretize.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"

#include <cmath>

namespace geoalgo
{
namespace
{

void appendPoint(Polyline3d& out, const gp_Pnt& p)
{
	out.xyz.push_back(static_cast<float>(p.X()));
	out.xyz.push_back(static_cast<float>(p.Y()));
	out.xyz.push_back(static_cast<float>(p.Z()));
}

bool discretizeCurveAdaptor(const BRepAdaptor_Curve& curve, const TessellateParams& params, Polyline3d& out)
{
	const double deflection = params.linearDeflectionRelative
		? params.linearDeflectionMm
		: params.linearDeflectionMm;
	GCPnts_UniformDeflection discretizer(curve, deflection, Standard_True);
	if (!discretizer.IsDone() || discretizer.NbPoints() < 2)
	{
		return false;
	}
	out.xyz.clear();
	out.xyz.reserve(static_cast<std::size_t>(discretizer.NbPoints()) * 3U);
	for (Standard_Integer i = 1; i <= discretizer.NbPoints(); ++i)
	{
		appendPoint(out, discretizer.Value(i));
	}
	return out.xyz.size() >= 6U;
}

} // namespace

bool discretizeEdge(
	const TopoDS_Edge& edge,
	const TessellateParams& params,
	Polyline3d& out,
	std::string* errMsg)
{
	if (edge.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "null edge";
		}
		return false;
	}
	const BRepAdaptor_Curve curve(edge);
	if (!discretizeCurveAdaptor(curve, params, out))
	{
		if (errMsg)
		{
			*errMsg = "edge discretization failed";
		}
		return false;
	}
	return true;
}

bool discretizeWire(
	const TopoDS_Wire& wire,
	const TessellateParams& params,
	Polyline3d& out,
	std::string* errMsg)
{
	if (wire.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "null wire";
		}
		return false;
	}
	out.xyz.clear();
	for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		Polyline3d segment;
		if (!discretizeEdge(edge, params, segment, errMsg))
		{
			return false;
		}
		if (segment.xyz.size() < 6U)
		{
			continue;
		}
		const std::size_t start = out.xyz.empty() ? 0U : 3U;
		for (std::size_t i = start; i < segment.xyz.size(); ++i)
		{
			out.xyz.push_back(segment.xyz[i]);
		}
	}
	if (out.xyz.size() < 6U)
	{
		if (errMsg)
		{
			*errMsg = "wire discretization empty";
		}
		return false;
	}
	return true;
}

bool discretizeShapeEdges(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::vector<Polyline3d>& out,
	std::string* errMsg)
{
	out.clear();
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		Polyline3d poly;
		if (!discretizeEdge(edge, params, poly, errMsg))
		{
			return false;
		}
		out.push_back(std::move(poly));
	}
	return !out.empty();
}

bool discretizeShapeEdges(
	const ShapeHandle& shapeHandle,
	const TessellateParams& params,
	std::vector<Polyline3d>& out,
	std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	return discretizeShapeEdges(shape, params, out, errMsg);
}

bool discretizeShapeEdgeByIndex(
	const ShapeHandle& shapeHandle,
	const int edgeIndex,
	const TessellateParams& params,
	Polyline3d& out,
	std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Edge edge;
	if (!shapeEdgeAtIndex(shape, edgeIndex, edge, errMsg))
	{
		return false;
	}
	return discretizeEdge(edge, params, out, errMsg);
}

} // namespace geoalgo
