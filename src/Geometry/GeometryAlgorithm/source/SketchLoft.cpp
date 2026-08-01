/// @file SketchLoft.cpp

#include "SketchLoft.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"
#include "detail/SketchCurveWireOcc.h"

#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

namespace geoalgo
{
namespace
{
bool makeClosedWire(const std::vector<float>& xyz, TopoDS_Wire& outWire, std::string* errMsg)
{
	double nx = 0, ny = 0, nz = 1;
	(void)estimatePolylinePlaneNormal(xyz, nx, ny, nz);
	return makeClosedWireFromPolylineMm(xyz, nx, ny, nz, outWire, errMsg);
}
} // namespace

bool sketchLoftPolylinesToHandle(const std::vector<float>& profileAXyzMm, const std::vector<float>& profileBXyzMm,
								 const SketchLoftParams& params, const ShapeHandle* baseOrNull, ShapeHandle& outShape,
								 std::string* errMsg)
{
	TopoDS_Wire w0, w1;
	if (!makeClosedWire(profileAXyzMm, w0, errMsg) || !makeClosedWire(profileBXyzMm, w1, errMsg))
		return false;
	try
	{
		BRepOffsetAPI_ThruSections loft(params.solid, Standard_False, 1.0e-4);
		loft.AddWire(w0);
		loft.AddWire(w1);
		loft.Build();
		if (!loft.IsDone())
		{
			if (errMsg)
				*errMsg = "ThruSections failed";
			return false;
		}
		TopoDS_Shape tool = loft.Shape();
		TopoDS_Shape baseNative;
		const TopoDS_Shape* basePtr = nullptr;
		if (baseOrNull && !baseOrNull->isNull())
		{
			if (!ShapeHandleAccess::nativeShape(*baseOrNull, &baseNative) || baseNative.IsNull())
			{
				if (errMsg)
					*errMsg = "invalid base ShapeHandle";
				return false;
			}
			basePtr = &baseNative;
		}
		TopoDS_Shape result;
		if (params.mode == SketchLoftMode::Boss)
		{
			if (!basePtr)
				result = tool;
			else if (!brepBooleanToShape(*basePtr, tool, BrepBooleanOp::Fuse, result, errMsg))
				return false;
		}
		else
		{
			if (!basePtr)
			{
				if (errMsg)
					*errMsg = "LoftCut requires base solid";
				return false;
			}
			if (!brepBooleanToShape(*basePtr, tool, BrepBooleanOp::Cut, result, errMsg))
				return false;
		}
		outShape = ShapeHandleAccess::fromNativeShape(&result);
		return !outShape.isNull();
	}
	catch (...)
	{
		if (errMsg)
			*errMsg = "loft exception";
		return false;
	}
}

} // namespace geoalgo
