/// @file FaceBoundaryDiscretizer.cpp
/// @brief FaceBoundaryDiscretizer 实现

#include "FaceBoundaryDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"

#include <BRepTools.hxx>

namespace geoalgo
{
REGISTER_FEATURE_DISCRETIZER(FaceBoundaryDiscretizer);

std::vector<FeatureDiscretizerParamField> FaceBoundaryDiscretizer::paramFields() const
{
	return featureDiscretizerCommonParamFields();
}

bool FaceBoundaryDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
										 std::string* errMsg) const
{
	if (!validate(input, errMsg))
	{
		return false;
	}
	if (input.geometry.faceIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "FaceBoundary requires faceIndices";
		}
		return false;
	}

	TopoDS_Face face;
	if (!shapeFaceAtIndex(shape, input.geometry.faceIndices[0], face, errMsg))
	{
		return false;
	}
	const TopoDS_Wire wire = BRepTools::OuterWire(face);
	if (wire.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "face has no outer wire";
		}
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	Polyline3d poly;
	if (!discretizeWire(wire, detail::toTessellate(disc), poly, errMsg))
	{
		return false;
	}
	const std::vector<TopoDS_Face> contextFaces{face};
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;
	frameCtx.pathClosed = true;
	detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
	out.closed = true;
	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
