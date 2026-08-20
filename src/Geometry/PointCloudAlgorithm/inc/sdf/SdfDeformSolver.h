#ifndef POINTCLOUDALGORITHM_SDF_DEFORMSOLVER_H
#define POINTCLOUDALGORITHM_SDF_DEFORMSOLVER_H

/// @file SdfDeformSolver.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 粗：DDF/SDF 场 + 平滑；细：点-面 / 场

#include "RegistrationSdf.h"
#include "sdf/DistanceField.h"
#include "sdf/SdfNodeSampler.h"

#include <string>
#include <utility>
#include <vector>

namespace pclalgo
{
namespace sdf
{

/// meshEdges：焊点后网格边；有则加边长保持，抑制蜘蛛网长三角
bool runSdfDeform(std::vector<float>& xyzInOut, std::vector<float>& normalsInOut, DistanceField& field,
				  const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg,
				  const std::vector<std::pair<int, int>>* meshEdges = nullptr);

} // namespace sdf
} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SDF_DEFORMSOLVER_H
