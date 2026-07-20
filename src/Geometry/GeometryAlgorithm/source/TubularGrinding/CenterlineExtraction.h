#ifndef GEOMETRYALGORITHM_CENTERLINEEXTRACTION_H
#define GEOMETRYALGORITHM_CENTERLINEEXTRACTION_H

/// @file CenterlineExtraction.h
/// @brief CenterlineExtraction 接口

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{
bool runCenterlineExtraction(const IndexedMeshLite& mesh, const TubularGrindingParams& params,
							 std::vector<TubularCenterlineSample>& outSamples, int& outSectionFitFailCount,
							 std::string* errMsg, TubularCenterlinePcaAxis* outPcaAxis = nullptr);

} // namespace tg
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_CENTERLINEEXTRACTION_H
