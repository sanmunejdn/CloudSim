#pragma once

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API bool reconstructPoisson(
	const std::vector<float>& xyz,
	const std::vector<float>& normalsNxNyNz,
	std::vector<float>& triangleSoupOut,
	double spacingMm = 0.0,
	double smAngleDeg = 20.0,
	double smRadiusRel = 30.0,
	double smDistanceRel = 0.375,
	std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool reconstructScaleSpace(
	const std::vector<float>& xyz,
	std::vector<float>& triangleSoupOut,
	std::size_t smoothIterations = 4,
	double meshingRadiusMm = 0.0,
	std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool reconstructPoissonAuto(
	std::vector<float> xyz,
	std::vector<float>& triangleSoupOut,
	double voxelPrefilterMm,
	double outlierRemovalPercent = 5.0,
	std::string* errMsg = nullptr);

} // namespace pclalgo
