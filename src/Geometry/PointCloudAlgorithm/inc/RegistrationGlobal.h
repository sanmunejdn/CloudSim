#pragma once

#include "point_cloud_algorithm_global.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{

struct RigidRegisterRansacParams
{
	std::size_t maxFeaturePoints = 4000U;
	double featureVoxelMm = 0.0;
	double inlierDistanceMm = 0.0;
	double maxNormalAngleDeg = 45.0;
	std::size_t minInliers = 40U;
	int maxIterations = 5000;
	unsigned int fpfhNeighbors = 20U;
	double modelDiagMm = 0.0;
	double faceBandMm = 2.0;
	bool refineWithIcp = true;
	bool skipTranslationCap = false;
};

POINT_CLOUD_ALGORITHM_API bool rigidRegisterFeatureRansac(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& sourceNormalsNxNyNz,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormalsNxNyNz,
	Eigen::Isometry3d& sourceToTarget,
	double* inlierRatio,
	RigidRegisterRansacParams params,
	std::string* errMsg = nullptr);

} // namespace pclalgo
