#ifndef POINTCLOUDALGORITHM_REGISTRATIONNONRIGID_H
#define POINTCLOUDALGORITHM_REGISTRATIONNONRIGID_H

/// @file RegistrationNonRigid.h
/// @brief 非刚性 TPS 形变（控制点位移 / 对应点拟合）

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{
/**
 * 控制点位移驱动 TPS，原地变形 xyz
 * @param controlPointIndices 控制点在 xyz 中的下标
 * @param controlDisplacementXyz 每控制点 3 个 double 位移 mm
 * @param regularizationLambda 正则，默认 1e-6；过大则更刚
 * @return false：索引越界、控制点不足或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool tpsDeformFromControls(std::vector<float>& xyzInOut,
													 const std::vector<std::size_t>& controlPointIndices,
													 const double* controlDisplacementXyz, std::size_t numControls,
													 double regularizationLambda = 1e-6, std::string* errMsg = nullptr);

/**
 * 由 source↔target 对应点拟合 TPS，输出变形后的 source
 * @param correspondenceIndices 与 source 点一一对应的 target 下标（长度=source 点数）或实现约定子集
 * @return false：对应非法或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool tpsFitAndDeform(const std::vector<float>& sourceXyz, const std::vector<float>& targetXyz,
											   const std::vector<std::size_t>& correspondenceIndices,
											   std::vector<float>& sourceXyzDeformedOut,
											   double regularizationLambda = 1e-6, std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONNONRIGID_H
