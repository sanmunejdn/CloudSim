/// @file SketchPattern.cpp

#include "SketchPattern.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
bool linearPatternBodyToHandle(const ShapeHandle& seed, const SketchLinearPatternParams& params, ShapeHandle& outShape,
							   std::string* errMsg)
{
	if (params.count < 2)
	{
		if (errMsg)
			*errMsg = "pattern count must be >= 2";
		return false;
	}
	TopoDS_Shape base;
	if (!ShapeHandleAccess::nativeShape(seed, &base) || base.IsNull())
	{
		if (errMsg)
			*errMsg = "invalid seed solid";
		return false;
	}
	TopoDS_Shape result = base;
	const gp_Vec step(params.dxMm, params.dyMm, params.dzMm);
	if (step.Magnitude() < 1e-12)
	{
		if (errMsg)
			*errMsg = "pattern step degenerate";
		return false;
	}
	for (int i = 1; i < params.count; ++i)
	{
		gp_Trsf tr;
		tr.SetTranslation(step * static_cast<double>(i));
		BRepBuilderAPI_Transform xf(base, tr, Standard_True);
		if (!xf.IsDone())
		{
			if (errMsg)
				*errMsg = "pattern transform failed";
			return false;
		}
		TopoDS_Shape fused;
		if (!brepBooleanToShape(result, xf.Shape(), BrepBooleanOp::Fuse, fused, errMsg))
			return false;
		result = fused;
	}
	outShape = ShapeHandleAccess::fromNativeShape(&result);
	return !outShape.isNull();
}

bool mirrorBodyToHandle(const ShapeHandle& seed, const SketchMirror3dParams& params, ShapeHandle& outShape,
						std::string* errMsg)
{
	TopoDS_Shape base;
	if (!ShapeHandleAccess::nativeShape(seed, &base) || base.IsNull())
	{
		if (errMsg)
			*errMsg = "invalid seed solid";
		return false;
	}
	gp_Vec n(params.nx, params.ny, params.nz);
	if (n.Magnitude() < 1e-12)
	{
		if (errMsg)
			*errMsg = "mirror plane normal degenerate";
		return false;
	}
	gp_Trsf tr;
	tr.SetMirror(gp_Ax2(gp_Pnt(params.ox, params.oy, params.oz), gp_Dir(n)));
	BRepBuilderAPI_Transform xf(base, tr, Standard_True);
	if (!xf.IsDone())
	{
		if (errMsg)
			*errMsg = "mirror transform failed";
		return false;
	}
	TopoDS_Shape mirrored = xf.Shape();
	if (!params.keepOriginal)
	{
		outShape = ShapeHandleAccess::fromNativeShape(&mirrored);
		return !outShape.isNull();
	}
	TopoDS_Shape fused;
	if (!brepBooleanToShape(base, mirrored, BrepBooleanOp::Fuse, fused, errMsg))
		return false;
	outShape = ShapeHandleAccess::fromNativeShape(&fused);
	return !outShape.isNull();
}

} // namespace geoalgo
