/// @file SketchShell.cpp

#include "SketchShell.h"

#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
bool shellFacesToHandle(const ShapeHandle& base, const std::vector<int>& openFaceIndices, double thicknessMm,
						ShapeHandle& outShape, std::string* errMsg)
{
	if (std::abs(thicknessMm) < 1e-6)
	{
		if (errMsg)
			*errMsg = "shell thickness too small";
		return false;
	}
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(base, &shape) || shape.IsNull())
	{
		if (errMsg)
			*errMsg = "invalid base solid";
		return false;
	}
	TopTools_ListOfShape closingFaces;
	for (int idx : openFaceIndices)
	{
		TopoDS_Face face;
		if (!shapeFaceAtIndex(shape, idx, face, nullptr))
			continue;
		closingFaces.Append(face);
	}
	if (closingFaces.IsEmpty())
	{
		if (errMsg)
			*errMsg = "no valid faces for shell";
		return false;
	}
	try
	{
		BRepOffsetAPI_MakeThickSolid mk;
		mk.MakeThickSolidByJoin(shape, closingFaces, thicknessMm, 1.0e-3);
		if (!mk.IsDone())
		{
			if (errMsg)
				*errMsg = "MakeThickSolid failed";
			return false;
		}
		const TopoDS_Shape result = mk.Shape();
		outShape = ShapeHandleAccess::fromNativeShape(&result);
		return !outShape.isNull();
	}
	catch (...)
	{
		if (errMsg)
			*errMsg = "shell exception";
		return false;
	}
}

} // namespace geoalgo
