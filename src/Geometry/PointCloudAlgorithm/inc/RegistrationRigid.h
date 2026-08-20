#ifndef POINTCLOUDALGORITHM_REGISTRATIONRIGID_H
#define POINTCLOUDALGORITHM_REGISTRATIONRIGID_H

/// @file RegistrationRigid.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 刚体 ICP：点-点 / 点-面（同坐标系 xyz，mm）

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
/**
 * 点-点 ICP；可选法线门控（maxNormalAngleDeg>0 且提供法线时）
 * 输入须同一坐标系；本库不感知 worldMatrix
 * @param sourceToTarget 输出刚体；列向量 p' = T * p
 * @param rmseMm 可选均方根误差 mm
 * @param maxIterations 默认 40
 * @param convergenceTransMm 平移收敛阈值 mm，默认 0.01
 * @param maxPairDistanceMm 配对最大距离 mm；≤0 自动
 * @param icpMaxPoints 参与 ICP 最大点数，默认 4000
 * @param maxNormalAngleDeg 法线夹角上限 °；≤0 不门控
 * @return false：点数不足、无有效配对或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool
rigidRegisterIcp(const std::vector<float>& sourceXyz, const std::vector<float>& targetXyz,
				 Eigen::Isometry3d& sourceToTarget, double* rmseMm, int maxIterations = 40,
				 double convergenceTransMm = 0.01, double maxPairDistanceMm = 0.0, std::size_t icpMaxPoints = 4000,
				 std::string* errMsg = nullptr, const std::vector<float>* sourceNormalsNxNyNz = nullptr,
				 const std::vector<float>* targetNormalsNxNyNz = nullptr, double maxNormalAngleDeg = 0.0);

/**
 * 点-面 ICP（需源/目标法线）；精配常用
 * 参数语义同 rigidRegisterIcp
 * @return false：法线缺失/长度不符、无有效配对或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool
rigidRegisterPointToPlaneIcp(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormalsNxNyNz,
							 const std::vector<float>& targetXyz, const std::vector<float>& targetNormalsNxNyNz,
							 Eigen::Isometry3d& sourceToTarget, double* rmseMm, int maxIterations = 40,
							 double convergenceTransMm = 0.01, double maxPairDistanceMm = 0.0,
							 std::size_t icpMaxPoints = 4000, std::string* errMsg = nullptr,
							 double maxNormalAngleDeg = 0.0);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONRIGID_H
