/// @file SketchLoft.cpp

#include "SketchLoft.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

namespace geoalgo
{
namespace
{
constexpr double kEps = 1e-9;

bool makeClosedWire(const std::vector<float>& xyz, TopoDS_Wire& outWire, std::string* errMsg)
{
	if (xyz.size() < 9U || (xyz.size() % 3U) != 0U)
	{
		if (errMsg)
			*errMsg = "loft profile needs >=3 points";
		return false;
	}
	std::vector<gp_Pnt> pts;
	const std::size_t nIn = xyz.size() / 3U;
	for (std::size_t i = 0; i < nIn; ++i)
	{
		const gp_Pnt p(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
		if (!pts.empty() && pts.back().Distance(p) < kEps)
			continue;
		pts.push_back(p);
	}
	if (pts.size() >= 2U && pts.front().Distance(pts.back()) < 1e-6)
		pts.pop_back();
	if (pts.size() < 3U)
	{
		if (errMsg)
			*errMsg = "loft profile needs >=3 points";
		return false;
	}
	BRepBuilderAPI_MakePolygon poly;
	for (const auto& p : pts)
		poly.Add(p);
	poly.Add(pts.front());
	poly.Close();
	if (!poly.IsDone())
	{
		if (errMsg)
			*errMsg = "loft MakePolygon failed";
		return false;
	}
	outWire = poly.Wire();
	return !outWire.IsNull();
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
