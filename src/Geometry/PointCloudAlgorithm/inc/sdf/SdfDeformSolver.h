#ifndef POINTCLOUDALGORITHM_SDF_DEFORMSOLVER_H
#define POINTCLOUDALGORITHM_SDF_DEFORMSOLVER_H

/// @file SdfDeformSolver.h
/// @brief 粗：DDF/SDF 场 + 平滑；细：点-面 / 场

#include "RegistrationSdf.h"
#include "sdf/DistanceField.h"
#include "sdf/SdfNodeSampler.h"

#include <string>
#include <vector>

namespace pclalgo
{
namespace sdf
{

bool runSdfDeform(std::vector<float>& xyzInOut, std::vector<float>& normalsInOut, DistanceField& field,
				  const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg);

} // namespace sdf
} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SDF_DEFORMSOLVER_H
