#pragma once

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{

bool runCenterlineExtraction(
	const IndexedMeshLite& mesh,
	const TubularGrindingParams& params,
	std::vector<TubularCenterlineSample>& outSamples,
	int& outSectionFitFailCount,
	std::string* errMsg,
	TubularCenterlinePcaAxis* outPcaAxis = nullptr);

} // namespace tg
} // namespace geoalgo
