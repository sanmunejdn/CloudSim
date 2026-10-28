/// @file FaceOffsetCurveDiscretizer.cpp
/// @brief FaceOffsetCurveDiscretizer 实现（投影曲线；跨工件源边折线已在 primary/面系）

#include "FaceOffsetCurveDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizeParamUtils.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"
#include "detail/OccIncludes.h"

#include <Precision.hxx>
#include <cmath>

namespace geoalgo
{
namespace
{
enum class OffsetMode
{
	Normal,	 ///< 投影后沿面法向抬高
	Surface ///< 投影后在切平面内横向偏置再回投
};

OffsetMode parseOffsetMode(const nlohmann::json& params)
{
	const std::string mode = paramString(params, "offsetMode", "Normal");
	if (mode == "Surface" || mode == "surface")
	{
		return OffsetMode::Surface;
	}
	return OffsetMode::Normal;
}

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

// 点在 shape 全局系；必须先变到 Face Location 局部再 Project（与 FeatureDiscretizeFrame 一致）
bool projectPointOntoFace(const gp_Pnt& ptWorld, const TopoDS_Face& face, gp_Pnt& onSWorld, gp_Vec& nrmWorld)
{
	TopLoc_Location faceLoc;
	const Handle(Geom_Surface) surf = BRep_Tool::Surface(face, faceLoc);
	if (surf.IsNull())
	{
		return false;
	}

	const gp_Trsf& toGlobal = faceLoc.Transformation();
	gp_Trsf toLocal = toGlobal;
	toLocal.Invert();

	gp_Pnt ptLocal = ptWorld;
	ptLocal.Transform(toLocal);

	GeomAPI_ProjectPointOnSurf proj(ptLocal, surf);
	if (!proj.IsDone() || proj.NbPoints() < 1)
	{
		return false;
	}

	Standard_Real u = 0.0;
	Standard_Real v = 0.0;
	proj.LowerDistanceParameters(u, v);

	gp_Pnt pOnSurf;
	gp_Vec du;
	gp_Vec dv;
	surf->D1(u, v, pOnSurf, du, dv);
	pOnSurf.Transform(toGlobal);
	onSWorld = pOnSurf;

	gp_Vec nLocal = du.Crossed(dv);
	if (nLocal.SquareMagnitude() <= Precision::SquareConfusion())
	{
		return false;
	}
	nLocal.Normalize();
	if (face.Orientation() == TopAbs_REVERSED)
	{
		nLocal.Reverse();
	}
	nLocal.Transform(toGlobal);
	if (nLocal.Magnitude() < 1e-12)
	{
		return false;
	}
	nLocal.Normalize();
	nrmWorld = nLocal;
	return true;
}

bool projectPolylineOntoFace(const Polyline3d& poly, const TopoDS_Face& face, double offsetMm, OffsetMode mode,
							 Polyline3d& offsetPoly, std::vector<std::size_t>& segmentEnds, std::size_t& outOkCount,
							 std::string* errMsg)
{
	(void)errMsg;
	outOkCount = 0;
	const std::size_t n = poly.xyz.size() / 3U;
	offsetPoly.xyz.clear();
	segmentEnds.clear();
	offsetPoly.xyz.reserve(poly.xyz.size());

	gp_Pnt prevOnS;
	bool havePrev = false;
	bool inSegment = false;

	auto finishSegment = [&]()
	{
		if (inSegment && !offsetPoly.xyz.empty())
		{
			segmentEnds.push_back(offsetPoly.xyz.size() / 3U);
			inSegment = false;
		}
		havePrev = false;
	};

	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		gp_Pnt p(poly.xyz[b], poly.xyz[b + 1U], poly.xyz[b + 2U]);
		gp_Pnt onS;
		gp_Vec nrm;
		if (!projectPointOntoFace(p, face, onS, nrm))
		{
			finishSegment();
			continue;
		}
		++outOkCount;

		gp_Pnt outPt = onS;
		if (mode == OffsetMode::Normal)
		{
			outPt.Translate(nrm.Multiplied(offsetMm));
		}
		else
		{
			gp_Vec lateral(0.0, 0.0, 0.0);
			if (havePrev)
			{
				gp_Vec tan(prevOnS, onS);
				if (tan.Magnitude() > 1e-9)
				{
					tan.Normalize();
					lateral = nrm.Crossed(tan);
					if (lateral.Magnitude() > 1e-12)
					{
						lateral.Normalize();
						lateral.Multiply(offsetMm);
					}
					else
					{
						lateral = gp_Vec(0.0, 0.0, 0.0);
					}
				}
			}
			outPt.Translate(lateral);
			gp_Pnt reprojected;
			gp_Vec n2;
			if (projectPointOntoFace(outPt, face, reprojected, n2))
			{
				outPt = reprojected;
			}
		}

		offsetPoly.xyz.push_back(static_cast<float>(outPt.X()));
		offsetPoly.xyz.push_back(static_cast<float>(outPt.Y()));
		offsetPoly.xyz.push_back(static_cast<float>(outPt.Z()));
		inSegment = true;
		prevOnS = onS;
		havePrev = true;
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
	fields.push_back(
		doubleFeatureParamField("offsetMm", "Normal lift", "法向抬高（可选）", "mm", -1000.0, 1000.0, 0.01, 0.0, 10));
	fields.push_back(enumFeatureParamField("offsetMode", "Offset mode", "抬高模式", {"Normal", "Surface"},
										   {"法向抬高", "面内横向"}, {"Normal", "Surface"}, 0, 11));
	fields.push_back(boolFeatureParamField("flipNormal", "Flip normal", "翻转法向/横向", false, 12));
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
	std::string faceErr;
	if (!shapeFaceAtIndex(faceShape, faceIdx, face, &faceErr))
	{
		if (errMsg)
		{
			*errMsg = faceErr;
		}
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(ctx.input);
	Polyline3d poly;
	const bool usePreXformPoly = edgeSlot && !edgeSlot->polylineInPrimaryMm.empty();
	if (usePreXformPoly)
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
		std::string edgeErr;
		if (edgeIdx < 0 || !shapeEdgeAtIndex(edgeShape, edgeIdx, edge, &edgeErr))
		{
			if (errMsg)
			{
				*errMsg = edgeErr.empty() ? "FaceOffsetCurve missing edge" : edgeErr;
			}
			return false;
		}
		if (!discretizeEdge(edge, detail::toTessellate(disc), poly, errMsg))
		{
			return false;
		}
	}

	double offset = paramDouble(ctx.input.params, "offsetMm", 0.0);
	if (paramBool(ctx.input.params, "flipNormal", false))
	{
		offset = -offset;
	}
	const OffsetMode mode = parseOffsetMode(ctx.input.params);
	Polyline3d offsetPoly;
	std::vector<std::size_t> segmentEnds;
	std::size_t okCount = 0;
	const std::size_t srcCount = poly.xyz.size() / 3U;
	if (!projectPolylineOntoFace(poly, face, offset, mode, offsetPoly, segmentEnds, okCount, errMsg))
	{
		if (errMsg)
		{
			*errMsg = "FaceOffsetCurve projection produced no points (src=" + std::to_string(srcCount) +
					  ", projected=" + std::to_string(okCount) + ")";
		}
		return false;
	}

	const std::vector<TopoDS_Face> contextFaces{face};
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;

	if (segmentEnds.empty())
	{
		detail::appendPolylineToRawPath(offsetPoly, out, disc, true, frameCtx);
	}
	else
	{
		std::size_t start = 0;
		for (const std::size_t endExclusive : segmentEnds)
		{
			if (endExclusive <= start)
			{
				continue;
			}
			Polyline3d part;
			part.xyz.assign(offsetPoly.xyz.begin() + static_cast<std::ptrdiff_t>(start * 3U),
							offsetPoly.xyz.begin() + static_cast<std::ptrdiff_t>(endExclusive * 3U));
			if (!out.points.empty())
			{
				out.segmentEndExclusive.push_back(out.points.size());
			}
			detail::appendPolylineToRawPath(part, out, disc, true, frameCtx);
			start = endExclusive;
		}
		if (start * 3U < offsetPoly.xyz.size())
		{
			Polyline3d part;
			part.xyz.assign(offsetPoly.xyz.begin() + static_cast<std::ptrdiff_t>(start * 3U), offsetPoly.xyz.end());
			if (!out.points.empty())
			{
				out.segmentEndExclusive.push_back(out.points.size());
			}
			detail::appendPolylineToRawPath(part, out, disc, true, frameCtx);
		}
	}

	detail::applyPostDiscretizeResample(strategyId(), ctx.input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
