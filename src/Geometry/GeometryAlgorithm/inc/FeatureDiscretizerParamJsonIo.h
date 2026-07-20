#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZERPARAMJSONIO_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZERPARAMJSONIO_H

/// @file FeatureDiscretizerParamJsonIo.h
/// @brief FeatureDiscretizerParamJsonIo 接口

#include "geometry_algorithm_global.h"

#include "FeatureListDocument.h"

#include <optional>
#include <string>
#include <vector>

#include <json.hpp>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API std::optional<nlohmann::json> loadFeatureDiscretizerJsonFile(const std::string& resourceBaseDir,
																					const std::string& relativePath);

GEOMETRY_ALGORITHM_API std::vector<FeatureDiscretizerParamField>
parseFeatureSchemaFieldsFromJson(const nlohmann::json& schemaRoot);

GEOMETRY_ALGORITHM_API nlohmann::json parseFeatureDefaultParamsFromJson(const nlohmann::json& root);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZERPARAMJSONIO_H
