/// @file FaceOffsetCurveDiscretizer.cpp
/// @brief FaceOffsetCurveDiscretizer 实现（支持跨工件：源边折线已在 primary/面系）

#include "FaceOffsetCurveDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizeParamUtils.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"
#include "detail/OccIncludes.h"

#include <BRepAdaptor_Surface.hxx>

namespace geoalgo
{
namespace
{
bool resolveOffsetRoles(const FeatureGeometry& geometry, int& edgeIdx, int& faceIdx, std::string* errMsg)
{
	edgeIdx = -1;
	faceIdx = -1;
	if (const GeometryRef* edge = findGeometryRef(geometry, "sourceEdge"))
	{
		edgeIdx = edge->index;
	}
	if (const GeometryRef* face = findGeometryRef(geometry, "targetFace"))
	{
		faceIdx = face->index;
	}
	if (edgeIdx < 0 || faceIdx < 0)
	{
		const std::vector<int> edges = collectGeometryIndices(geometry, GeometryRefKind::Edge);
		const std::vector<int> faces = collectGeometryIndices(geometry, GeometryRefKind::Face);
		if (edgeIdx < 0 && !edges.empty())
		{
			edgeIdx = edges.front();
		}
		if (faceIdx < 0 && !faces.empty())
		{
			faceIdx = faces.front();
		}
	}
	if (edgeIdx < 0 || faceIdx < 0)
	{
		if (errMsg)
		{
			*errMsg = "FaceOffsetCurve requires sourceEdge and targetFace";
		}
		return false;
	}
	return true;
}

bool projectPolylineOntoFace(const Polyline3d& poly, const TopoDS_Face& face, double offsetMm, Polyline3d& offsetPoly,
							 std::string* errMsg)
{
	(void)errMsg;
	const BRepAdaptor_Surface surf(face);
	const std::size_t n = poly.xyz.size() / 3U;
	offsetPoly.xyz.reserve(poly.xyz.size());
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		gp_Pnt p(poly.xyz[b], poly.xyz[b + 1U], poly.xyz[b + 2U]);
		GeomAPI_ProjectPointOnSurf proj(p, BRep_Tool::Surface(face));
		if (proj.NbPoints() < 1)
		{
			continue;
		}
		Standard_Real u = 0.0;
		Standard_Real v = 0.0;
		proj.LowerDistanceParameters(u, v);
		gp_Pnt onS;
		gp_Vec du;
		gp_Vec dv;
		surf.D1(u, v, onS, du, dv);
		gp_Vec nrm = du.Crossed(dv);
		if (face.Orientation() == TopAbs_REVERSED)
		{
			nrm.Reverse();
		}
		if (nrm.Magnitude() < 1e-12)
		{
			continue;
		}
		nrm.Normalize();
		onS.Translate(nrm.Multiplied(offsetMm));
		offsetPoly.xyz.push_back(static_cast<float>(onS.X()));
		offsetPoly.xyz.push_back(static_cast<float>(onS.Y()));
		offsetPoly.xyz.push_back(static_cast<float>(onS.Z()));
	}
	return !offsetPoly.xyz.empty();
}

const ShapeSlot* findSlotByRole(const FeatureDiscretizeContext& ctx, const std::string& role)
{
	for (const ShapeSlot& slot : ctx.shapeSlots)
	{
		if (slot.role == role)
		{
			return &slot;
		}
	}
	return nullptr;
}

bool handleToTopo(const ShapeHandle& handle, TopoDS_Shape& out)
{
	return ShapeHandleAccess::nativeShape(handle, &out) && !out.IsNull();
}
} // namespace

REGISTER_FEATURE_DISCRETIZER(FaceOffsetCurveDiscretizer);

std::vector<FeatureDiscretizerParamField> FaceOffsetCurveDiscretizer::paramFields() const
{
	std::vector<FeatureDiscretizerParamField> fields = featureDiscretizerCommonParamFields();
	fields.push_back(doubleFeatureParamField("offsetMm", "Offset", "偏置距离", "mm", -1000.0, 1000.0, 0.01, 0.0, 10));
	return fields;
}

bool FaceOffsetCurveDiscretizer::validate(const FeatureDiscretizeInput& input, std::string* errMsg) const
{
	if (input.strategyId != strategyId())
	{
		if (errMsg)
		{
			*errMsg = "strategyId mismatch";
		}
		return false;
	}
	int edgeIdx = -1;
	int faceIdx = -1;
	return resolveOffsetRoles(input.geometry, edgeIdx, faceIdx, errMsg);
}

bool FaceOffsetCurveDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input,
											RawPath& out, std::string* errMsg) const
{
	FeatureDiscretizeContext ctx;
	ctx.primaryShapeHandle = ShapeHandleAccess::fromNativeShape(&shape);
	ctx.input = input;
	ctx.primaryWorkpiece = input.workpiece;
	int edgeIdx = -1;
	int faceIdx = -1;
	if (!resolveOffsetRoles(input.geometry, edgeIdx, faceIdx, errMsg))
	{
		return false;
	}
	ShapeSlot edgeSlot;
	edgeSlot.role = "sourceEdge";
	edgeSlot.shapeHandle = ctx.primaryShapeHandle;
	edgeSlot.localIndex = edgeIdx;
	ShapeSlot faceSlot;
	faceSlot.role = "targetFace";
	faceSlot.shapeHandle = ctx.primaryShapeHandle;
	faceSlot.localIndex = faceIdx;
	ctx.shapeSlots.push_back(std::move(edgeSlot));
	ctx.shapeSlots.push_back(std::move(faceSlot));
	return discretizeEx(ctx, out, errMsg);
}

bool FaceOffsetCurveDiscretizer::discretizeEx(const FeatureDiscretizeContext& ctx, RawPath& out,
											  std::string* errMsg) const
{
	if (!validate(ctx.input, errMsg))
	{
		return false;
	}

	const ShapeSlot* faceSlot = findSlotByRole(ctx, "targetFace");
	const ShapeSlot* edgeSlot = findSlotByRole(ctx, "sourceEdge");
	int edgeIdx = -1;
	int faceIdx = -1;
	(void)resolveOffsetRoles(ctx.input.geometry, edgeIdx, faceIdx, nullptr);
	if (faceSlot)
	{
		faceIdx = faceSlot->localIndex >= 0 ? faceSlot->localIndex : faceIdx;
	}
	if (edgeSlot)
	{
		edgeIdx = edgeSlot->localIndex >= 0 ? edgeSlot->localIndex : edgeIdx;
	}

	TopoDS_Shape faceShape;
	if (faceSlot && handleToTopo(faceSlot->shapeHandle, faceShape))
	{
	}
	else if (!handleToTopo(ctx.primaryShapeHandle, faceShape))
	{
		if (errMsg)
		{
			*errMsg = "FaceOffsetCurve missing targetFace shape";
		}
		return false;
	}
	if (faceIdx < 0)
	{
		if (errMsg)
		{
			*errMsg = "FaceOffsetCurve missing targetFace index";
		}
		return false;
	}

	TopoDS_Face face;
	if (!shapeFaceAtIndex(faceShape, faceIdx, face, errMsg))
	{
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(ctx.input);
	Polyline3d poly;
	if (edgeSlot && !edgeSlot->polylineInPrimaryMm.empty())
	{
		poly.xyz = edgeSlot->polylineInPrimaryMm;
	}
	else
	{
		TopoDS_Shape edgeShape;
		if (!(edgeSlot && handleToTopo(edgeSlot->shapeHandle, edgeShape)) &&
			!handleToTopo(ctx.primaryShapeHandle, edgeShape))
		{
			edgeShape = faceShape;
		}
		TopoDS_Edge edge;
		if (edgeIdx < 0 || !shapeEdgeAtIndex(edgeShape, edgeIdx, edge, errMsg))
		{
			return false;
		}
		if (!discretizeEdge(edge, detail::toTessellate(disc), poly, errMsg))
		{
			return false;
		}
	}

	const double offset = paramDouble(ctx.input.params, "offsetMm", 0.0);
	Polyline3d offsetPoly;
	if (!projectPolylineOntoFace(poly, face, offset, offsetPoly, errMsg))
	{
		if (errMsg && errMsg->empty())
		{
			*errMsg = "FaceOffsetCurve projection produced no points";
		}
		return false;
	}

	const std::vector<TopoDS_Face> contextFaces{face};
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;
	detail::appendPolylineToRawPath(offsetPoly, out, disc, true, frameCtx);

	detail::applyPostDiscretizeResample(strategyId(), ctx.input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
