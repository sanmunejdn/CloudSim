/// @file FaceIntersectionDiscretizer.cpp
/// @brief FaceIntersectionDiscretizer 实现（同件/跨件：异件面已在 buildContext 变到 primary）

#include "FaceIntersectionDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizerRegistry.h"
#include "Intersection.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"

namespace geoalgo
{
namespace
{
bool handleToTopo(const ShapeHandle& handle, TopoDS_Shape& out)
{
	return ShapeHandleAccess::nativeShape(handle, &out) && !out.IsNull();
}

bool resolveFaceFromSlotOrShape(const ShapeSlot* slot, const TopoDS_Shape& fallbackShape, int fallbackIndex,
								TopoDS_Face& outFace, std::string* errMsg)
{
	TopoDS_Shape shape = fallbackShape;
	int index = fallbackIndex;
	if (slot)
	{
		TopoDS_Shape slotShape;
		if (handleToTopo(slot->shapeHandle, slotShape))
		{
			shape = slotShape;
		}
		if (slot->localIndex >= 0)
		{
			index = slot->localIndex;
		}
	}
	if (shape.IsNull() || index < 0)
	{
		if (errMsg)
		{
			*errMsg = "FaceIntersection missing face";
		}
		return false;
	}
	return shapeFaceAtIndex(shape, index, outFace, errMsg);
}

bool appendIntersectionResult(const IntersectionResult& result, const std::vector<TopoDS_Face>& contextFaces,
							  const DiscretizeParams& disc, RawPath& out, std::string* errMsg)
{
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;

	if (!result.curves.empty())
	{
		for (const Polyline3d& poly : result.curves)
		{
			if (!out.points.empty())
			{
				out.segmentEndExclusive.push_back(out.points.size());
			}
			detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
		}
	}
	else if (!result.points.empty())
	{
		if (!out.points.empty())
		{
			out.segmentEndExclusive.push_back(out.points.size());
		}
		for (const IntersectionHit& hit : result.points)
		{
			RawPathPoint rp;
			rp.positionMm = hit.positionMm;
			out.points.push_back(rp);
		}
	}

	if (out.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "intersection produced no geometry";
		}
		return false;
	}
	return true;
}
} // namespace

REGISTER_FEATURE_DISCRETIZER(FaceIntersectionDiscretizer);

std::vector<FeatureDiscretizerParamField> FaceIntersectionDiscretizer::paramFields() const
{
	return featureDiscretizerCommonParamFields();
}

bool FaceIntersectionDiscretizer::validate(const FeatureDiscretizeInput& input, std::string* errMsg) const
{
	if (!IFeatureDiscretizer::validate(input, errMsg))
	{
		return false;
	}
	if (collectGeometryIndices(input.geometry, GeometryRefKind::Face).size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "FaceIntersection requires two Face refs";
		}
		return false;
	}
	return true;
}

bool FaceIntersectionDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input,
											 RawPath& out, std::string* errMsg) const
{
	FeatureDiscretizeContext ctx;
	ctx.primaryShapeHandle = ShapeHandleAccess::fromNativeShape(&shape);
	ctx.input = input;
	ctx.primaryWorkpiece = input.workpiece;
	const std::vector<int> faceIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Face);
	if (faceIndices.size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "FaceIntersection requires two Face refs";
		}
		return false;
	}
	for (std::size_t i = 0; i < 2U; ++i)
	{
		ShapeSlot slot;
		slot.shapeHandle = ctx.primaryShapeHandle;
		slot.localIndex = faceIndices[i];
		ctx.shapeSlots.push_back(std::move(slot));
	}
	return discretizeEx(ctx, out, errMsg);
}

bool FaceIntersectionDiscretizer::discretizeEx(const FeatureDiscretizeContext& ctx, RawPath& out,
											   std::string* errMsg) const
{
	if (!validate(ctx.input, errMsg))
	{
		return false;
	}

	TopoDS_Shape primaryShape;
	(void)handleToTopo(ctx.primaryShapeHandle, primaryShape);

	const std::vector<int> faceIndices = collectGeometryIndices(ctx.input.geometry, GeometryRefKind::Face);
	const ShapeSlot* slot0 = ctx.shapeSlots.size() > 0U ? &ctx.shapeSlots[0] : nullptr;
	const ShapeSlot* slot1 = ctx.shapeSlots.size() > 1U ? &ctx.shapeSlots[1] : nullptr;

	TopoDS_Face f1;
	TopoDS_Face f2;
	const int idx0 = faceIndices.size() > 0U ? faceIndices[0] : -1;
	const int idx1 = faceIndices.size() > 1U ? faceIndices[1] : -1;
	if (!resolveFaceFromSlotOrShape(slot0, primaryShape, idx0, f1, errMsg) ||
		!resolveFaceFromSlotOrShape(slot1, primaryShape, idx1, f2, errMsg))
	{
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(ctx.input);
	IntersectionParams ip;
	ip.discretizeCurves = true;
	ip.curveDisc = detail::toTessellate(disc);
	IntersectionResult result;
	if (!intersectFaces(f1, f2, ip, result, errMsg))
	{
		return false;
	}

	const std::vector<TopoDS_Face> contextFaces{f1, f2};
	if (!appendIntersectionResult(result, contextFaces, disc, out, errMsg))
	{
		return false;
	}
	detail::applyPostDiscretizeResample(strategyId(), ctx.input.params, out);
	return true;
}

} // namespace geoalgo
