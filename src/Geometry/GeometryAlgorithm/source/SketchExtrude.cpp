/// @file SketchExtrude.cpp
/// @brief 移植 FreeCAD FeatureExtrude：BRepPrimAPI_MakePrism + Fuse/Cut

#include "SketchExtrude.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <cmath>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace geoalgo
{
namespace
{
constexpr double kDraftAngleEpsilon = 1e-4;
constexpr double kSideFaceDotThreshold = 0.9;

bool applyDraftAngle(TopoDS_Shape& solid, const SketchExtrudeParams& params, const gp_Dir& extrudeDir)
{
	if (std::abs(params.draftAngleDeg) <= kDraftAngleEpsilon)
		return true;

	const double angleRad = params.draftAngleDeg * M_PI / 180.0;
	gp_Dir draftDir = extrudeDir;
	const gp_Pln neutral(gp_Pnt(params.originX, params.originY, params.originZ),
						 gp_Dir(params.normalX, params.normalY, params.normalZ));

	BRepOffsetAPI_DraftAngle draft(solid);
	bool anyAdded = false;
	for (TopExp_Explorer exp(solid, TopAbs_FACE); exp.More(); exp.Next())
	{
		TopoDS_Face face = TopoDS::Face(exp.Current());
		BRepAdaptor_Surface faceSurface(face, Standard_True);
		if (faceSurface.GetType() != GeomAbs_Plane)
			continue;
		gp_Pln facePlane = faceSurface.Plane();
		gp_Dir faceNormal = facePlane.Axis().Direction();
		if (face.Orientation() == TopAbs_REVERSED)
			faceNormal.Reverse();
		if (std::abs(faceNormal.Dot(draftDir)) > kSideFaceDotThreshold)
			continue;
		draft.Add(face, draftDir, angleRad, neutral, Standard_True);
		if (draft.AddDone())
			anyAdded = true;
		else
			draft.Remove(face);
	}
	if (!anyAdded)
		return true;
	draft.Build();
	if (!draft.IsDone())
		return false;
	solid = draft.Shape();
	return !solid.IsNull();
}

bool makeFaceFromClosedPolyline(const std::vector<float>& xyz, TopoDS_Face& outFace, std::string* errMsg)
{
	if (xyz.size() < 9U || (xyz.size() % 3U) != 0U)
	{
		if (errMsg)
			*errMsg = "polyline needs >=3 points";
		return false;
	}
	BRepBuilderAPI_MakePolygon poly;
	const std::size_t n = xyz.size() / 3U;
	for (std::size_t i = 0; i < n; ++i)
	{
		poly.Add(gp_Pnt(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
	}
	const gp_Pnt p0(xyz[0], xyz[1], xyz[2]);
	const gp_Pnt pLast(xyz[(n - 1) * 3], xyz[(n - 1) * 3 + 1], xyz[(n - 1) * 3 + 2]);
	if (p0.Distance(pLast) > 1e-6)
	{
		poly.Add(p0);
	}
	poly.Close();
	if (!poly.IsDone())
	{
		if (errMsg)
			*errMsg = "MakePolygon failed";
		return false;
	}
	BRepBuilderAPI_MakeFace mkFace(poly.Wire(), Standard_True);
	if (!mkFace.IsDone())
	{
		if (errMsg)
			*errMsg = "MakeFace from wire failed";
		return false;
	}
	outFace = mkFace.Face();
	return true;
}

bool makeFaceFromProfileAndHoles(const std::vector<float>& outerXyz,
								 const std::vector<std::vector<float>>& holePolylinesXyzMm, TopoDS_Face& outFace,
								 std::string* errMsg)
{
	TopoDS_Face outerFace;
	if (!makeFaceFromClosedPolyline(outerXyz, outerFace, errMsg))
		return false;
	BRepBuilderAPI_MakeFace mkFace(outerFace);
	for (const auto& hole : holePolylinesXyzMm)
	{
		if (hole.size() < 9U)
			continue;
		BRepBuilderAPI_MakePolygon poly;
		const std::size_t n = hole.size() / 3U;
		for (std::size_t i = 0; i < n; ++i)
			poly.Add(gp_Pnt(hole[i * 3], hole[i * 3 + 1], hole[i * 3 + 2]));
		const gp_Pnt p0(hole[0], hole[1], hole[2]);
		const gp_Pnt pLast(hole[(n - 1) * 3], hole[(n - 1) * 3 + 1], hole[(n - 1) * 3 + 2]);
		if (p0.Distance(pLast) > 1e-6)
			poly.Add(p0);
		poly.Close();
		if (!poly.IsDone())
			continue;
		mkFace.Add(poly.Wire());
	}
	if (!mkFace.IsDone())
	{
		if (errMsg)
			*errMsg = "MakeFace with holes failed";
		return false;
	}
	outFace = mkFace.Face();
	return true;
}

bool sketchExtrudeProfileNative(const TopoDS_Shape& profileFaceOrWire, const SketchExtrudeParams& params,
								const TopoDS_Shape* baseOrNull, TopoDS_Shape& outShape, std::string* errMsg)
{
	double lengthMm = 0.0;
	if (params.endCondition == SketchExtrudeEndCondition::ThroughAll)
	{
		if (!baseOrNull || baseOrNull->IsNull())
		{
			if (errMsg)
				*errMsg = "ThroughAll requires base solid";
			return false;
		}
		gp_Dir dir(params.normalX, params.normalY, params.normalZ);
		if (params.reversed)
			dir.Reverse();
		Bnd_Box box;
		BRepBndLib::Add(*baseOrNull, box);
		if (box.IsVoid())
		{
			if (errMsg)
				*errMsg = "ThroughAll: base bbox empty";
			return false;
		}
		double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
		box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		const gp_Pnt O(params.originX, params.originY, params.originZ);
		const gp_XYZ d = dir.XYZ();
		double tMax = -1e300;
		const double xs[2] = {xmin, xmax};
		const double ys[2] = {ymin, ymax};
		const double zs[2] = {zmin, zmax};
		for (double x : xs)
			for (double y : ys)
				for (double z : zs)
				{
					const double t = gp_XYZ(x - O.X(), y - O.Y(), z - O.Z()).Dot(d);
					if (t > tMax)
						tMax = t;
				}
		if (tMax <= 1e-6)
		{
			if (errMsg)
				*errMsg = "ThroughAll: solid not ahead of sketch";
			return false;
		}
		// 略超出包围盒，避免布尔残留薄片
		const double diag = box.SquareExtent() > 0.0 ? std::sqrt(box.SquareExtent()) : tMax;
		lengthMm = tMax + std::max(1.0, 0.01 * diag);
	}
	else if (!resolveSketchExtrudeLengthMm(params, lengthMm, errMsg))
	{
		return false;
	}

	TopoDS_Face face;
	if (profileFaceOrWire.ShapeType() == TopAbs_FACE)
	{
		face = TopoDS::Face(profileFaceOrWire);
	}
	else if (profileFaceOrWire.ShapeType() == TopAbs_WIRE)
	{
		BRepBuilderAPI_MakeFace mkFace(TopoDS::Wire(profileFaceOrWire), Standard_True);
		if (!mkFace.IsDone())
		{
			if (errMsg)
				*errMsg = "MakeFace from wire failed";
			return false;
		}
		face = mkFace.Face();
	}
	else
	{
		TopExp_Explorer ex(profileFaceOrWire, TopAbs_FACE);
		if (!ex.More())
		{
			if (errMsg)
				*errMsg = "profile has no face/wire";
			return false;
		}
		face = TopoDS::Face(ex.Current());
	}

	gp_Dir dir(params.normalX, params.normalY, params.normalZ);
	if (params.reversed)
		dir.Reverse();

	TopoDS_Shape tool;
	// 对称：正反半棱柱各自拔模后再 Fuse（整块侧壁跨中性面时 DraftAngle 常无效）
	if (params.endCondition == SketchExtrudeEndCondition::MidPlane)
	{
		const double half = 0.5 * lengthMm;
		const gp_Vec fwdVec(dir.XYZ() * half);
		const gp_Vec bwdVec(dir.XYZ() * (-half));
		BRepPrimAPI_MakePrism fwdPrism(face, fwdVec, Standard_True, Standard_True);
		BRepPrimAPI_MakePrism bwdPrism(face, bwdVec, Standard_True, Standard_True);
		if (!fwdPrism.IsDone() || !bwdPrism.IsDone())
		{
			if (errMsg)
				*errMsg = "MidPlane MakePrism failed";
			return false;
		}
		TopoDS_Shape fwdTool = fwdPrism.Shape();
		TopoDS_Shape bwdTool = bwdPrism.Shape();
		(void)applyDraftAngle(fwdTool, params, dir);
		(void)applyDraftAngle(bwdTool, params, dir.Reversed());
		if (!brepBooleanToShape(fwdTool, bwdTool, BrepBooleanOp::Fuse, tool, errMsg))
			return false;
	}
	else
	{
		const gp_Vec vec(dir.XYZ() * lengthMm);
		BRepPrimAPI_MakePrism prism(face, vec, Standard_False, Standard_True);
		if (!prism.IsDone())
		{
			if (errMsg)
				*errMsg = "MakePrism failed";
			return false;
		}
		tool = prism.Shape();
		(void)applyDraftAngle(tool, params, dir); // 失败保留棱柱
	}
	if (params.mode == SketchExtrudeMode::Pad)
	{
		if (!baseOrNull || baseOrNull->IsNull())
		{
			outShape = tool;
			return true;
		}
		return brepBooleanToShape(*baseOrNull, tool, BrepBooleanOp::Fuse, outShape, errMsg);
	}
	if (!baseOrNull || baseOrNull->IsNull())
	{
		if (errMsg)
			*errMsg = "Pocket requires base solid";
		return false;
	}
	return brepBooleanToShape(*baseOrNull, tool, BrepBooleanOp::Cut, outShape, errMsg);
}
} // namespace

bool resolveSketchExtrudeLengthMm(const SketchExtrudeParams& params, double& outLengthMm, std::string* errMsg)
{
	return resolveSketchExtrudeLengthMm(params, outLengthMm, static_cast<const ShapeHandle*>(nullptr), errMsg);
}

bool resolveSketchExtrudeLengthMm(const SketchExtrudeParams& params, double& outLengthMm, const ShapeHandle* baseOrNull,
								  std::string* errMsg)
{
	if (params.endCondition == SketchExtrudeEndCondition::ThroughAll)
	{
		if (!baseOrNull || baseOrNull->isNull())
		{
			if (errMsg)
				*errMsg = "ThroughAll requires base solid";
			return false;
		}
		TopoDS_Shape baseNative;
		if (!ShapeHandleAccess::nativeShape(*baseOrNull, &baseNative) || baseNative.IsNull())
		{
			if (errMsg)
				*errMsg = "ThroughAll: invalid base ShapeHandle";
			return false;
		}
		gp_Dir dir(params.normalX, params.normalY, params.normalZ);
		if (params.reversed)
			dir.Reverse();
		Bnd_Box box;
		BRepBndLib::Add(baseNative, box);
		if (box.IsVoid())
		{
			if (errMsg)
				*errMsg = "ThroughAll: base bbox empty";
			return false;
		}
		double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
		box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		const gp_Pnt O(params.originX, params.originY, params.originZ);
		const gp_XYZ d = dir.XYZ();
		double tMax = -1e300;
		const double xs[2] = {xmin, xmax};
		const double ys[2] = {ymin, ymax};
		const double zs[2] = {zmin, zmax};
		for (double x : xs)
			for (double y : ys)
				for (double z : zs)
				{
					const double t = gp_XYZ(x - O.X(), y - O.Y(), z - O.Z()).Dot(d);
					if (t > tMax)
						tMax = t;
				}
		if (tMax <= 1e-6)
		{
			if (errMsg)
				*errMsg = "ThroughAll: solid not ahead of sketch";
			return false;
		}
		const double diag = box.SquareExtent() > 0.0 ? std::sqrt(box.SquareExtent()) : tMax;
		outLengthMm = tMax + std::max(1.0, 0.01 * diag);
		return true;
	}

	gp_Dir dir(params.normalX, params.normalY, params.normalZ);
	if (params.reversed)
		dir.Reverse();
	const gp_Pnt O(params.originX, params.originY, params.originZ);

	if (params.endCondition == SketchExtrudeEndCondition::UpToVertex && params.hasUpToVertex)
	{
		const gp_Pnt V(params.upToVertexX, params.upToVertexY, params.upToVertexZ);
		const double t = gp_Vec(O, V).Dot(gp_Vec(dir));
		if (t <= 1e-6)
		{
			if (errMsg)
				*errMsg = "UpToVertex: vertex not ahead of sketch";
			return false;
		}
		outLengthMm = t;
		return true;
	}

	if ((params.endCondition == SketchExtrudeEndCondition::UpToFace
		 || params.endCondition == SketchExtrudeEndCondition::OffsetFromFace)
		&& params.hasUpToFace)
	{
		gp_Dir faceN(params.upNormalX, params.upNormalY, params.upNormalZ);
		const double denom = dir.Dot(faceN);
		if (std::abs(denom) < 1e-9)
		{
			if (errMsg)
				*errMsg = "UpToFace: extrude direction parallel to face";
			return false;
		}
		const gp_Pnt P(params.upOriginX, params.upOriginY, params.upOriginZ);
		double t = gp_Vec(O, P).Dot(faceN) / denom;
		if (t <= 1e-6)
		{
			if (errMsg)
				*errMsg = "UpToFace: target face not ahead of sketch";
			return false;
		}
		if (params.endCondition == SketchExtrudeEndCondition::OffsetFromFace)
			t += params.offsetFromFaceMm;
		if (t <= 1e-6)
		{
			if (errMsg)
				*errMsg = "OffsetFromFace: effective length <= 0";
			return false;
		}
		outLengthMm = t;
		return true;
	}

	if (params.lengthMm <= 1e-9)
	{
		if (errMsg)
			*errMsg = "lengthMm must be > 0";
		return false;
	}
	outLengthMm = params.lengthMm;
	return true;
}

bool sketchExtrudePolylineToHandle(const std::vector<float>& closedPolylineXyzMm, const SketchExtrudeParams& params,
								   const ShapeHandle* baseOrNull, ShapeHandle& outShape, std::string* errMsg)
{
	TopoDS_Face face;
	if (params.holePolylinesXyzMm.empty())
	{
		if (!makeFaceFromClosedPolyline(closedPolylineXyzMm, face, errMsg))
			return false;
	}
	else if (!makeFaceFromProfileAndHoles(closedPolylineXyzMm, params.holePolylinesXyzMm, face, errMsg))
	{
		return false;
	}
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
	if (!sketchExtrudeProfileNative(face, params, basePtr, result, errMsg))
	{
		return false;
	}
	outShape = ShapeHandleAccess::fromNativeShape(&result);
	return !outShape.isNull();
}

} // namespace geoalgo
