#pragma once

#include "IFeatureDiscretizer.h"

namespace geoalgo
{

class FaceBoundaryDiscretizer final : public IFeatureDiscretizer
{
public:
	std::string strategyId() const override { return "FaceBoundary"; }
	std::string displayNameZh() const override { return "面外轮廓"; }
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
