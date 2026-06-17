#pragma once

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{

bool runTrajectoryTemplates(
	const std::vector<TubularPipeSegment>& segments,
	const std::vector<TubularCenterlineSample>& centerlineSamples,
	const TubularGrindingParams& params,
	std::vector<TubularTemplatePoint>& outPoints,
	std::string* errMsg);

} // namespace tg
} // namespace geoalgo
