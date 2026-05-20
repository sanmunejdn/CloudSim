#pragma once

#include "point_cloud_algorithm_global.h"

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API bool rigidRegisterIcp(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& targetXyz,
	Eigen::Isometry3d& sourceToTarget,
	double* rmseMm,
	int maxIterations = 40,
	double convergenceTransMm = 0.01,
	double maxPairDistanceMm = 0.0,
	std::size_t icpMaxPoints = 4000,
	std::string* errMsg = nullptr);

} // namespace pclalgo
