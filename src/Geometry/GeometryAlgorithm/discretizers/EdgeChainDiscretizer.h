#ifndef GEOMETRYALGORITHM_EDGECHAINDISCRETIZER_H
#define GEOMETRYALGORITHM_EDGECHAINDISCRETIZER_H

/// @file EdgeChainDiscretizer.h
/// @brief EdgeChainDiscretizer 接口

#include "IFeatureDiscretizer.h"

namespace geoalgo
{
class EdgeChainDiscretizer final : public IFeatureDiscretizer
{
public:
	std::string strategyId() const override { return "EdgeChain"; }
	std::string displayNameZh() const override { return "边链"; }
	GeometryAffinity affinity() const override { return GeometryAffinity::Line; }
	MergePolicy mergePolicy() const override { return MergePolicy::LineConnectivity; }

	std::vector<FeatureDiscretizerParamField> paramFields() const override;

	bool discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
					std::string* errMsg = nullptr) const override;
};

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_EDGECHAINDISCRETIZER_H
