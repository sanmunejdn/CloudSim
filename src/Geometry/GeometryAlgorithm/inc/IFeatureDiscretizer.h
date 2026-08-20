#ifndef GEOMETRYALGORITHM_IFEATUREDISCRETIZER_H
#define GEOMETRYALGORITHM_IFEATUREDISCRETIZER_H

/// @file IFeatureDiscretizer.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 特征离散策略插件接口（strategyId / mergePolicy / discretize）

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
