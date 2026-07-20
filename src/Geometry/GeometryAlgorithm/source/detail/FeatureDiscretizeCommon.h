#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZECOMMON_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZECOMMON_H

/// @file FeatureDiscretizeCommon.h
/// @brief FeatureDiscretizeCommon 接口

#include "Discretize.h"
#include "FeatureDiscretizeParamUtils.h"
#include "FeatureListDocument.h"
#include "detail/FeatureDiscretizeFrame.h"

#include <string>

namespace geoalgo
{
namespace detail
{
DiscretizeParams buildDiscretizeParamsFromInput(const FeatureDiscretizeInput& input);

TessellateParams toTessellate(const DiscretizeParams& p);

bool resampleRawPathByStep(RawPath& path, double stepMm);

void appendRawPath(const RawPath& part, RawPath& out);

bool shouldResampleAfterDiscretize(const std::string& strategyId, double stepMm);

void applyPostDiscretizeResample(const std::string& strategyId, const nlohmann::json& params, RawPath& path);

} // namespace detail
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZECOMMON_H
