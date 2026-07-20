#ifndef GEOMETRYALGORITHM_IFEATUREDISCRETIZERCONFIG_H
#define GEOMETRYALGORITHM_IFEATUREDISCRETIZERCONFIG_H

/// @file IFeatureDiscretizerConfig.h
/// @brief IFeatureDiscretizerConfig 接口

#include "geometry_algorithm_global.h"

#include "FeatureListDocument.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace geoalgo
{
class GEOMETRY_ALGORITHM_API IFeatureDiscretizerConfig
{
public:
	virtual ~IFeatureDiscretizerConfig() = default;

	virtual std::string strategyId() const = 0;
	virtual std::string jsonRelativePath() const = 0;
	virtual std::vector<FeatureDiscretizerParamField> paramFields() const = 0;
	virtual nlohmann::json defaultParams() const = 0;
};

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_IFEATUREDISCRETIZERCONFIG_H
