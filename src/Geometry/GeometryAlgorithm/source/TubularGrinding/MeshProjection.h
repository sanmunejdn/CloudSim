#ifndef GEOMETRYALGORITHM_MESHPROJECTION_H
#define GEOMETRYALGORITHM_MESHPROJECTION_H

/// @file MeshProjection.h
/// @brief MeshProjection 接口

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{
bool runMeshProjection(const IndexedMeshLite& mesh, const std::vector<TubularTemplatePoint>& templatePoints,
					   const TubularGrindingParams& params, std::vector<TubularProjectedPoint>& outPoints,
					   double& outHitRate, std::string* errMsg);

bool runPointCloudProjection(const std::vector<float>& pointXyz,
							 const std::vector<TubularTemplatePoint>& templatePoints,
							 const TubularGrindingParams& params, std::vector<TubularProjectedPoint>& outPoints,
							 double& outHitRate, std::string* errMsg);

} // namespace tg
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHPROJECTION_H
