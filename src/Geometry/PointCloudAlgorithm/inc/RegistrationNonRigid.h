#ifndef POINTCLOUDALGORITHM_REGISTRATIONNONRIGID_H
#define POINTCLOUDALGORITHM_REGISTRATIONNONRIGID_H

/// @file RegistrationNonRigid.h
/// @brief RegistrationNonRigid 接口

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{
POINT_CLOUD_ALGORITHM_API bool tpsDeformFromControls(std::vector<float>& xyzInOut,
													 const std::vector<std::size_t>& controlPointIndices,
													 const double* controlDisplacementXyz, std::size_t numControls,
													 double regularizationLambda = 1e-6, std::string* errMsg = nullptr);

POINT_CLOUD_ALGORITHM_API bool tpsFitAndDeform(const std::vector<float>& sourceXyz, const std::vector<float>& targetXyz,
											   const std::vector<std::size_t>& correspondenceIndices,
											   std::vector<float>& sourceXyzDeformedOut,
											   double regularizationLambda = 1e-6, std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONNONRIGID_H
