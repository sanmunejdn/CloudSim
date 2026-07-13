#pragma once

#include "FeatureListDocument.h"
#include "geometry_algorithm_global.h"

#include <json.hpp>

#include <optional>
#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API std::optional<nlohmann::json> loadFeatureDiscretizerJsonFile(
	const std::string& resourceBaseDir,
	const std::string& relativePath);

GEOMETRY_ALGORITHM_API std::vector<FeatureDiscretizerParamField> parseFeatureSchemaFieldsFromJson(
	const nlohmann::json& schemaRoot);

GEOMETRY_ALGORITHM_API nlohmann::json parseFeatureDefaultParamsFromJson(const nlohmann::json& root);

} // namespace geoalgo
