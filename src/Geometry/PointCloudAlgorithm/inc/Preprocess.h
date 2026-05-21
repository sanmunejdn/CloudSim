#pragma once

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API bool estimateNormalsPca(
	const std::vector<float>& xyz,
	std::vector<float>& normalsOut,
	unsigned int kNeighbors = 12,
	std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool estimateNormalsJet(
	const std::vector<float>& xyz,
	std::vector<float>& normalsOut,
	unsigned int kNeighbors = 12,
	unsigned int degreeFitting = 2,
	std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool orientNormalsMst(
	std::vector<float>& xyz,
	std::vector<float>& normalsInOut,
	unsigned int kNeighbors = 12,
	std::vector<float>* rgbaInOut = nullptr,
	std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool removeOutliers(
	std::vector<float>& xyzInOut,
	double removalPercent = 5.0,
	unsigned int kNeighbors = 24,
	std::vector<float>* normalsInOut = nullptr,
	std::vector<float>* rgbaInOut = nullptr,
	std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool smoothBilateral(
	std::vector<float>& xyzInOut,
	std::vector<float>* normalsInOut = nullptr,
	std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool preprocessForReconstruction(
	std::vector<float>& xyzInOut,
	std::vector<float>& normalsOut,
	double voxelPrefilterMm,
	double outlierRemovalPercent,
	std::string* errMsg = nullptr);

} // namespace pclalgo
