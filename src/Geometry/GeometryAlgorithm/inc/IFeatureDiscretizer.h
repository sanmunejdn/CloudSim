#ifndef GEOMETRYALGORITHM_IFEATUREDISCRETIZER_H
#define GEOMETRYALGORITHM_IFEATUREDISCRETIZER_H

/// @file IFeatureDiscretizer.h
/// @brief IFeatureDiscretizer 接口

#include "geometry_algorithm_global.h"

#include "FeatureListDocument.h"

#include <string>

#include <TopoDS_Shape.hxx>

namespace geoalgo
{
class GEOMETRY_ALGORITHM_API IFeatureDiscretizer
{
public:
	virtual ~IFeatureDiscretizer() = default;

	virtual std::string strategyId() const = 0;
	virtual std::string displayNameZh() const = 0;
	virtual GeometryAffinity affinity() const = 0;
	virtual MergePolicy mergePolicy() const = 0;

	virtual std::vector<FeatureDiscretizerParamField> paramFields() const = 0;

	virtual bool discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
							std::string* errMsg = nullptr) const = 0;

	virtual bool validate(const FeatureDiscretizeInput& input, std::string* errMsg = nullptr) const;
};

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_IFEATUREDISCRETIZER_H
