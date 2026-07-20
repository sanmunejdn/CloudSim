#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZEPARAMUTILS_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZEPARAMUTILS_H

/// @file FeatureDiscretizeParamUtils.h
/// @brief FeatureDiscretizeParamUtils 接口

#include "geometry_algorithm_global.h"

#include "FeatureListDocument.h"

#include <string>

#include <json.hpp>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API double paramDouble(const nlohmann::json& params, const std::string& key, double defaultValue);

GEOMETRY_ALGORITHM_API std::string paramString(const nlohmann::json& params, const std::string& key,
											   const std::string& defaultValue);

GEOMETRY_ALGORITHM_API bool paramBool(const nlohmann::json& params, const std::string& key, bool defaultValue);

GEOMETRY_ALGORITHM_API int paramInt(const nlohmann::json& params, const std::string& key, int defaultValue);

GEOMETRY_ALGORITHM_API DiscretizeParams buildDiscretizeParams(const nlohmann::json& params);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZEPARAMUTILS_H
