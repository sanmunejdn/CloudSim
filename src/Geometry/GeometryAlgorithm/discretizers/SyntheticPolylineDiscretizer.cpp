#include "SyntheticPolylineDiscretizer.h"

#include "FeatureDiscretizerRegistry.h"
#include "detail/FeatureDiscretizeCommon.h"

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
	if (input.geometry.polylineXyz.size() < 6U)
	{
		if (errMsg)
		{
			*errMsg = "SyntheticPolyline requires at least 2 points";
		}
		return false;
	}
	return true;
}

bool SyntheticPolylineDiscretizer::discretize(
	const TopoDS_Shape& shape,
	const FeatureDiscretizeInput& input,
	RawPath& out,
	std::string* errMsg) const
{
	(void)shape;
	if (!validate(input, errMsg))
	{
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	Polyline3d poly;
	poly.xyz = input.geometry.polylineXyz;
	detail::appendPolylineToRawPath(poly, out, disc, true);
	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
