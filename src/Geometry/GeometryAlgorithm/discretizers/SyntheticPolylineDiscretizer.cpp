/// @file SyntheticPolylineDiscretizer.cpp
/// @brief SyntheticPolylineDiscretizer 实现

#include "SyntheticPolylineDiscretizer.h"

#include "FeatureDiscretizerRegistry.h"
#include "detail/FeatureDiscretizeCommon.h"

#include <TopoDS_Face.hxx>
#include <cmath>

namespace geoalgo
{
REGISTER_FEATURE_DISCRETIZER(SyntheticPolylineDiscretizer);

std::vector<FeatureDiscretizerParamField> SyntheticPolylineDiscretizer::paramFields() const
{
	return featureDiscretizerCommonParamFields();
}

bool SyntheticPolylineDiscretizer::validate(const FeatureDiscretizeInput& input, std::string* errMsg) const
{
	if (input.strategyId != strategyId())
	{
		if (errMsg)
		{
			*errMsg = "strategyId mismatch";
		}
		return false;
	}
	const std::vector<float> xyz = collectPolylineXyz(input.geometry);
	if (xyz.size() < 6U || (xyz.size() % 3U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "SyntheticPolyline requires at least 2 points";
		}
		return false;
	}
	for (float v : xyz)
	{
		if (!std::isfinite(v))
		{
			if (errMsg)
			{
				*errMsg = "SyntheticPolyline contains non-finite coordinates";
			}
			return false;
		}
	}
	constexpr float kMinSeg = 1e-6f;
	bool hasUsefulSeg = false;
	const std::size_t n = xyz.size() / 3U;
	for (std::size_t i = 1; i < n; ++i)
	{
		const std::size_t b0 = (i - 1U) * 3U;
		const std::size_t b1 = i * 3U;
		const float dx = xyz[b1] - xyz[b0];
		const float dy = xyz[b1 + 1U] - xyz[b0 + 1U];
		const float dz = xyz[b1 + 2U] - xyz[b0 + 2U];
		if (dx * dx + dy * dy + dz * dz > kMinSeg * kMinSeg)
		{
			hasUsefulSeg = true;
			break;
		}
	}
	if (!hasUsefulSeg)
	{
		if (errMsg)
		{
			*errMsg = "SyntheticPolyline segments are degenerate";
		}
		return false;
	}
	return true;
}

bool SyntheticPolylineDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input,
											  RawPath& out, std::string* errMsg) const
{
	(void)shape;
	if (!validate(input, errMsg))
	{
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	Polyline3d poly;
	poly.xyz = collectPolylineXyz(input.geometry);
	const std::vector<int> faceIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Face);
	detail::PolylineFrameContext frameCtx;
	std::vector<TopoDS_Face> contextFaces;
	if (!faceIndices.empty() && !shape.IsNull())
	{
		contextFaces = detail::collectContextFaces(shape, {}, faceIndices, nullptr);
		if (!contextFaces.empty())
		{
			frameCtx.faces = &contextFaces;
			frameCtx.normalConvention = detail::FaceNormalConvention::LineReverseFace;
		}
	}
	detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
