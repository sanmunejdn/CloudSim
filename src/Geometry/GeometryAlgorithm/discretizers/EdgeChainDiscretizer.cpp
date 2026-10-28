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
	const std::vector<int> edgeIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Edge);
	const std::vector<int> faceIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Face);
	if (edgeIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "EdgeChain requires Edge refs";
		}
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	const TessellateParams tess = detail::toTessellate(disc);
	const std::vector<TopoDS_Face> contextFaces = detail::collectContextFaces(shape, edgeIndices, faceIndices, errMsg);
	if (!faceIndices.empty() && contextFaces.empty())
	{
		return false;
	}
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = contextFaces.empty() ? nullptr : &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;

	if (edgeIndices.size() == 1)
	{
		TopoDS_Edge edge;
		if (!shapeEdgeAtIndex(shape, edgeIndices[0], edge, errMsg))
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
		if (!fuseStepEdgesToPolyline(input.workpiece.stepPathUtf8, edgeIndices, tess, poly, errMsg))
		{
			return false;
		}
		detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
	}

	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
