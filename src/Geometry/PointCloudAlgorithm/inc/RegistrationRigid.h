#ifndef POINTCLOUDALGORITHM_REGISTRATIONRIGID_H
#define POINTCLOUDALGORITHM_REGISTRATIONRIGID_H

/// @file RegistrationRigid.h
/// @brief RegistrationRigid 接口

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
POINT_CLOUD_ALGORITHM_API bool
rigidRegisterIcp(const std::vector<float>& sourceXyz, const std::vector<float>& targetXyz,
				 Eigen::Isometry3d& sourceToTarget, double* rmseMm, int maxIterations = 40,
				 double convergenceTransMm = 0.01, double maxPairDistanceMm = 0.0, std::size_t icpMaxPoints = 4000,
				 std::string* errMsg = nullptr, const std::vector<float>* sourceNormalsNxNyNz = nullptr,
				 const std::vector<float>* targetNormalsNxNyNz = nullptr, double maxNormalAngleDeg = 0.0);

POINT_CLOUD_ALGORITHM_API bool
rigidRegisterPointToPlaneIcp(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormalsNxNyNz,
							 const std::vector<float>& targetXyz, const std::vector<float>& targetNormalsNxNyNz,
							 Eigen::Isometry3d& sourceToTarget, double* rmseMm, int maxIterations = 40,
							 double convergenceTransMm = 0.01, double maxPairDistanceMm = 0.0,
							 std::size_t icpMaxPoints = 4000, std::string* errMsg = nullptr,
							 double maxNormalAngleDeg = 0.0);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONRIGID_H
