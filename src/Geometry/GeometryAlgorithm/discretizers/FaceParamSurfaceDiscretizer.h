#pragma once

#include "IFeatureDiscretizer.h"

namespace geoalgo
{

class FaceParamSurfaceDiscretizer final : public IFeatureDiscretizer
{
public:
	std::string strategyId() const override { return "FaceParamSurface"; }
	std::string displayNameZh() const override { return "参数面扫描"; }
	GeometryAffinity affinity() const override { return GeometryAffinity::Face; }
	MergePolicy mergePolicy() const override { return MergePolicy::FaceUnion; }

	std::vector<FeatureDiscretizerParamField> paramFields() const override;

	bool discretize(
		const TopoDS_Shape& shape,
		const FeatureDiscretizeInput& input,
		RawPath& out,
		std::string* errMsg = nullptr) const override;
};

} // namespace geoalgo
