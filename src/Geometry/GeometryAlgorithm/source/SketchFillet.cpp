/// @file SketchFillet.cpp

#include "SketchFillet.h"

#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
namespace
{
bool nativeBase(const ShapeHandle& base, TopoDS_Shape& out, std::string* errMsg)
{
	if (!ShapeHandleAccess::nativeShape(base, &out) || out.IsNull())
	{
		if (errMsg)
			*errMsg = "invalid base solid";
		return false;
	}
	return true;
}
} // namespace

bool filletEdgesToHandle(const ShapeHandle& base, const std::vector<int>& edgeIndices, double radiusMm,
						 ShapeHandle& outShape, std::string* errMsg)
{
	if (radiusMm < 1e-6)
	{
		if (errMsg)
			*errMsg = "fillet radius too small";
		return false;
	}
	TopoDS_Shape shape;
	if (!nativeBase(base, shape, errMsg))
		return false;
	try
	{
		BRepFilletAPI_MakeFillet mk(shape);
		int added = 0;
		for (int idx : edgeIndices)
		{
			TopoDS_Edge edge;
			if (!shapeEdgeAtIndex(shape, idx, edge, nullptr))
				continue;
			mk.Add(radiusMm, edge);
			++added;
		}
		if (added == 0)
		{
			if (errMsg)
				*errMsg = "no valid edges for fillet";
			return false;
		}
		mk.Build();
		if (!mk.IsDone())
		{
			if (errMsg)
				*errMsg = "fillet failed (radius too large?)";
			return false;
		}
		const TopoDS_Shape result = mk.Shape();
		outShape = ShapeHandleAccess::fromNativeShape(&result);
		return !outShape.isNull();
	}
	catch (...)
	{
		if (errMsg)
			*errMsg = "fillet exception";
		return false;
	}
}

bool chamferEdgesToHandle(const ShapeHandle& base, const std::vector<int>& edgeIndices, double distanceMm,
						  ShapeHandle& outShape, std::string* errMsg)
{
	if (distanceMm < 1e-6)
	{
		if (errMsg)
			*errMsg = "chamfer distance too small";
		return false;
	}
	TopoDS_Shape shape;
	if (!nativeBase(base, shape, errMsg))
		return false;
	try
	{
		BRepFilletAPI_MakeChamfer mk(shape);
		TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
		TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);
		int added = 0;
		for (int idx : edgeIndices)
		{
			TopoDS_Edge edge;
			if (!shapeEdgeAtIndex(shape, idx, edge, nullptr))
				continue;
			const int mapIdx = edgeFaceMap.FindIndex(edge);
			if (mapIdx <= 0)
				continue;
			const TopTools_ListOfShape& faces = edgeFaceMap(mapIdx);
			if (faces.IsEmpty())
				continue;
			mk.Add(distanceMm, distanceMm, edge, TopoDS::Face(faces.First()));
			++added;
		}
		if (added == 0)
		{
			if (errMsg)
				*errMsg = "no valid edges for chamfer";
			return false;
		}
		mk.Build();
		if (!mk.IsDone())
		{
			if (errMsg)
				*errMsg = "chamfer failed";
			return false;
		}
		const TopoDS_Shape result = mk.Shape();
		outShape = ShapeHandleAccess::fromNativeShape(&result);
		return !outShape.isNull();
	}
	catch (...)
	{
		if (errMsg)
			*errMsg = "chamfer exception";
		return false;
	}
}

} // namespace geoalgo
