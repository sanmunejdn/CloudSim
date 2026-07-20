#ifndef GEOMETRYALGORITHM_PIPESEGMENTATION_H
#define GEOMETRYALGORITHM_PIPESEGMENTATION_H

/// @file PipeSegmentation.h
/// @brief PipeSegmentation 接口

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{
bool runPipeSegmentation(const IndexedMeshLite& mesh, const TubularGrindingParams& params,
						 std::vector<TubularPipeSegment>& outSegments, std::vector<TubularCrossSectionRing>& outRings,
						 std::vector<int>& outFaceSegmentId, int& outJunctionFaceCount, int& outRegionCountBeforeFilter,
						 std::string* errMsg, std::vector<Vec3>* outFaceLocalAxes = nullptr);

} // namespace tg
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_PIPESEGMENTATION_H
