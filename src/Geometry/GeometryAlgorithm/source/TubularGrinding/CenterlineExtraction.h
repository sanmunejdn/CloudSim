#pragma once

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{

bool runCenterlineExtraction(
	const IndexedMeshLite& mesh,
	const std::vector<TubularPipeSegment>& segments,
	const TubularGrindingParams& params,
	std::vector<TubularCenterlineSample>& outSamples,
	int& outSectionFitFailCount,
	std::string* errMsg);

} // namespace tg
} // namespace geoalgo
