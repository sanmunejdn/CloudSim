#pragma once

#include "FeatureListDocument.h"
#include "geometry_algorithm_global.h"

#include <json.hpp>

#include <string>
#include <vector>

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
