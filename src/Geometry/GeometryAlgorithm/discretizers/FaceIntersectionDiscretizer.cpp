/// @file FaceIntersectionDiscretizer.cpp
/// @brief FaceIntersectionDiscretizer 实现

#include "FaceIntersectionDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizerRegistry.h"
#include "Intersection.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"

namespace geoalgo
{
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
	if (!validate(input, errMsg))
	{
		return false;
	}

	const std::vector<int> faceIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Face);
	TopoDS_Face f1;
	TopoDS_Face f2;
	if (!shapeFaceAtIndex(shape, faceIndices[0], f1, errMsg) || !shapeFaceAtIndex(shape, faceIndices[1], f2, errMsg))
	{
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	IntersectionParams ip;
	ip.discretizeCurves = true;
	ip.curveDisc = detail::toTessellate(disc);
	IntersectionResult result;
	if (!intersectFaces(f1, f2, ip, result, errMsg))
	{
		return false;
	}

	const std::vector<TopoDS_Face> contextFaces{f1, f2};
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;

	if (!result.curves.empty())
	{
		for (const Polyline3d& poly : result.curves)
		{
			detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
		}
	}
	else if (!result.points.empty())
	{
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

	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return true;
}

} // namespace geoalgo
