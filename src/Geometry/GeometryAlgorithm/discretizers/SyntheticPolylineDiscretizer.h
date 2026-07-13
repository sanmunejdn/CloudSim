#pragma once

#include "IFeatureDiscretizer.h"

namespace geoalgo
{

class SyntheticPolylineDiscretizer final : public IFeatureDiscretizer
{
public:
	std::string strategyId() const override { return "SyntheticPolyline"; }
	std::string displayNameZh() const override { return "合成折线"; }
	GeometryAffinity affinity() const override { return GeometryAffinity::Any; }
	MergePolicy mergePolicy() const override { return MergePolicy::None; }

	std::vector<FeatureDiscretizerParamField> paramFields() const override;

	bool discretize(
		const TopoDS_Shape& shape,
		const FeatureDiscretizeInput& input,
		RawPath& out,
		std::string* errMsg = nullptr) const override;

	bool validate(const FeatureDiscretizeInput& input, std::string* errMsg = nullptr) const override;
};

} // namespace geoalgo
