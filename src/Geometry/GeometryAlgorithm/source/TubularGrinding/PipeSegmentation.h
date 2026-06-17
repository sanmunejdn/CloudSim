#pragma once

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{

bool runPipeSegmentation(
	const IndexedMeshLite& mesh,
	const TubularGrindingParams& params,
	std::vector<TubularPipeSegment>& outSegments,
	std::vector<TubularCrossSectionRing>& outRings,
	std::vector<int>& outFaceSegmentId,
	int& outJunctionFaceCount,
	int& outRegionCountBeforeFilter,
	std::string* errMsg);

} // namespace tg
} // namespace geoalgo
