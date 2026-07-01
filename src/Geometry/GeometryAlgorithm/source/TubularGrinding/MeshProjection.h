#pragma once

#include "TubularGrindingCommon.h"

namespace geoalgo
{
namespace tg
{

bool runMeshProjection(
	const IndexedMeshLite& mesh,
	const std::vector<TubularTemplatePoint>& templatePoints,
	const TubularGrindingParams& params,
	std::vector<TubularProjectedPoint>& outPoints,
	double& outHitRate,
	std::string* errMsg);

bool runPointCloudProjection(
	const std::vector<float>& pointXyz,
	const std::vector<TubularTemplatePoint>& templatePoints,
	const TubularGrindingParams& params,
	std::vector<TubularProjectedPoint>& outPoints,
	double& outHitRate,
	std::string* errMsg);

} // namespace tg
} // namespace geoalgo
