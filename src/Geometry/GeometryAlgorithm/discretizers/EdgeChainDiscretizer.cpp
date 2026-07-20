/// @file EdgeChainDiscretizer.cpp
/// @brief EdgeChainDiscretizer 实现

#include "EdgeChainDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeQuery.h"
#include "WireOps.h"
#include "detail/FeatureDiscretizeCommon.h"

namespace geoalgo
{
REGISTER_FEATURE_DISCRETIZER(EdgeChainDiscretizer);

std::vector<FeatureDiscretizerParamField> EdgeChainDiscretizer::paramFields() const
{
	return featureDiscretizerCommonParamFields();
}

bool EdgeChainDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
									  std::string* errMsg) const
{
	if (!validate(input, errMsg))
	{
		return false;
	}
	if (input.geometry.edgeIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "EdgeChain requires edgeIndices";
		}
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	const TessellateParams tess = detail::toTessellate(disc);
	const std::vector<TopoDS_Face> contextFaces =
		detail::collectContextFaces(shape, input.geometry.edgeIndices, input.geometry.faceIndices, errMsg);
	if (!input.geometry.faceIndices.empty() && contextFaces.empty())
	{
		return false;
	}
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = contextFaces.empty() ? nullptr : &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;

	if (input.geometry.edgeIndices.size() == 1)
	{
		TopoDS_Edge edge;
		if (!shapeEdgeAtIndex(shape, input.geometry.edgeIndices[0], edge, errMsg))
		{
			return false;
		}
		Polyline3d poly;
		if (!discretizeEdge(edge, tess, poly, errMsg))
		{
			return false;
		}
		detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
	}
	else
	{
		Polyline3d poly;
		if (!fuseStepEdgesToPolyline(input.workpiece.stepPathUtf8, input.geometry.edgeIndices, tess, poly, errMsg))
		{
			return false;
		}
		detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
	}

	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
