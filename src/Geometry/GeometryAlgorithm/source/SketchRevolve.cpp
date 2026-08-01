/// @file SketchRevolve.cpp

#include "SketchRevolve.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"
#include "detail/SketchCurveWireOcc.h"

#include <BRepPrimAPI_MakeRevol.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace geoalgo
{
namespace
{
bool makeClosedFace(const std::vector<float>& xyz, TopoDS_Face& outFace, std::string* errMsg)
{
	double nx = 0, ny = 0, nz = 1;
	(void)estimatePolylinePlaneNormal(xyz, nx, ny, nz);
	return makeClosedFaceFromPolylineMm(xyz, nx, ny, nz, outFace, errMsg);
}
} // namespace

bool sketchRevolvePolylineToHandle(const std::vector<float>& profilePolylineXyzMm, const SketchRevolveParams& params,
								   const ShapeHandle* baseOrNull, ShapeHandle& outShape, std::string* errMsg)
{
	TopoDS_Face face;
	if (!makeClosedFace(profilePolylineXyzMm, face, errMsg))
		return false;

	gp_Vec dir(params.axisDx, params.axisDy, params.axisDz);
	if (dir.Magnitude() < 1e-12)
	{
		if (errMsg)
			*errMsg = "revolve axis direction degenerate";
		return false;
	}
	const gp_Ax1 axis(gp_Pnt(params.axisOx, params.axisOy, params.axisOz), gp_Dir(dir));
	double angle = params.angleDeg * M_PI / 180.0;
	if (std::abs(angle) < 1e-9)
		angle = 2.0 * M_PI;
	if (std::abs(std::abs(angle) - 2.0 * M_PI) < 1e-6)
		angle = 2.0 * M_PI;

	try
	{
		BRepPrimAPI_MakeRevol revol(face, axis, angle, Standard_True);
		if (!revol.IsDone())
		{
			if (errMsg)
				*errMsg = "MakeRevol failed";
			return false;
		}
		TopoDS_Shape tool = revol.Shape();
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
		if (params.mode == SketchRevolveMode::Boss)
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
					*errMsg = "RevolveCut requires base solid";
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
			*errMsg = "revolve exception";
		return false;
	}
}

} // namespace geoalgo
