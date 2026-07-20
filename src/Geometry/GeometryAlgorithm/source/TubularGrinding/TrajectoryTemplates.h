#ifndef GEOMETRYALGORITHM_TRAJECTORYTEMPLATES_H
#define GEOMETRYALGORITHM_TRAJECTORYTEMPLATES_H

/// @file TrajectoryTemplates.h
/// @brief TrajectoryTemplates 接口

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{
bool runTrajectoryTemplates(const std::vector<TubularPipeSegment>& segments,
							const std::vector<TubularCenterlineSample>& centerlineSamples,
							const TubularGrindingParams& params, std::vector<TubularTemplatePoint>& outPoints,
							std::string* errMsg);

} // namespace tg
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_TRAJECTORYTEMPLATES_H
