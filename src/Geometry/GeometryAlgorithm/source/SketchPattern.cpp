/// @file SketchPattern.cpp

#include "SketchPattern.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>

#include <functional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace geoalgo
{
namespace
{
bool fuseTransformedCopies(const TopoDS_Shape& seedNative, TopoDS_Shape& result, int count,
						   const std::function<gp_Trsf(int)>& makeTrsf, std::string* errMsg)
{
	for (int i = 1; i < count; ++i)
	{
		BRepBuilderAPI_Transform xf(seedNative, makeTrsf(i), Standard_True);
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
	return true;
}
} // namespace

bool featureContributionSeed(const ShapeHandle& tipAfter, const ShapeHandle& tipBefore, ShapeHandle& outSeed,
							 std::string* errMsg)
{
	outSeed = {};
	if (tipAfter.isNull())
	{
		if (errMsg)
			*errMsg = "tipAfter is null";
		return false;
	}
	if (tipBefore.isNull())
	{
		outSeed = tipAfter;
		return true;
	}
	TopoDS_Shape afterN, beforeN;
	if (!ShapeHandleAccess::nativeShape(tipAfter, &afterN) || afterN.IsNull())
	{
		if (errMsg)
			*errMsg = "invalid tipAfter";
		return false;
	}
	if (!ShapeHandleAccess::nativeShape(tipBefore, &beforeN) || beforeN.IsNull())
	{
		outSeed = tipAfter;
		return true;
	}
	TopoDS_Shape contrib;
	std::string cutErr;
	if (!brepBooleanToShape(afterN, beforeN, BrepBooleanOp::Cut, contrib, &cutErr) || contrib.IsNull())
	{
		// Cut 失败时回退整 tipAfter，避免阵列断链
		outSeed = tipAfter;
		if (errMsg)
			*errMsg = cutErr.empty() ? "contribution Cut failed; fallback tipAfter" : cutErr;
		return true;
	}
	outSeed = ShapeHandleAccess::fromNativeShape(&contrib);
	return !outSeed.isNull();
}

bool linearPatternBodyToHandle(const ShapeHandle& seed, const SketchLinearPatternParams& params, ShapeHandle& outShape,
							   std::string* errMsg, const ShapeHandle* fuseOnto)
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
	const gp_Vec step(params.dxMm, params.dyMm, params.dzMm);
	if (step.Magnitude() < 1e-12)
	{
		if (errMsg)
			*errMsg = "pattern step degenerate";
		return false;
	}

	TopoDS_Shape result;
	if (fuseOnto && !fuseOnto->isNull())
	{
		if (!ShapeHandleAccess::nativeShape(*fuseOnto, &result) || result.IsNull())
		{
			if (errMsg)
				*errMsg = "invalid fuseOnto tip";
			return false;
		}
	}
	else
	{
		result = base;
	}

	auto makeTrsf = [&](int i) {
		gp_Trsf tr;
		tr.SetTranslation(step * static_cast<double>(i));
		return tr;
	};
	if (!fuseTransformedCopies(base, result, params.count, makeTrsf, errMsg))
		return false;
	outShape = ShapeHandleAccess::fromNativeShape(&result);
	return !outShape.isNull();
}

bool circularPatternBodyToHandle(const ShapeHandle& seed, const SketchCircularPatternParams& params,
								 ShapeHandle& outShape, std::string* errMsg, const ShapeHandle* fuseOnto)
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
	gp_Vec dir(params.axisDx, params.axisDy, params.axisDz);
	if (dir.Magnitude() < 1e-12)
	{
		if (errMsg)
			*errMsg = "circular pattern axis degenerate";
		return false;
	}
	const gp_Ax1 axis(gp_Pnt(params.axisOx, params.axisOy, params.axisOz), gp_Dir(dir));
	const double stepRad = (params.angleDeg / static_cast<double>(params.count)) * M_PI / 180.0;

	TopoDS_Shape result;
	if (fuseOnto && !fuseOnto->isNull())
	{
		if (!ShapeHandleAccess::nativeShape(*fuseOnto, &result) || result.IsNull())
		{
			if (errMsg)
				*errMsg = "invalid fuseOnto tip";
			return false;
		}
	}
	else
	{
		result = base;
	}

	auto makeTrsf = [&](int i) {
		gp_Trsf tr;
		tr.SetRotation(axis, stepRad * static_cast<double>(i));
		return tr;
	};
	if (!fuseTransformedCopies(base, result, params.count, makeTrsf, errMsg))
		return false;
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
