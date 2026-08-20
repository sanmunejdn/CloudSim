#ifndef POINTCLOUDALGORITHM_REGISTRATIONGLOBAL_H
#define POINTCLOUDALGORITHM_REGISTRATIONGLOBAL_H

/// @file RegistrationGlobal.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 全局粗配准：FPFH + RANSAC + Kabsch（可选点-面 ICP 精修）

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
/// FPFH-RANSAC 粗配参数；距离类字段为 0 时按 modelDiag 自动
struct RigidRegisterRansacParams
{
	std::size_t maxFeaturePoints = 4000U; ///< 特征点上限
	double featureVoxelMm = 0.0;		  ///< 特征体素 mm；0=自动
	double inlierDistanceMm = 0.0;		  ///< 内点距离 mm；0=自动
	double maxNormalAngleDeg = 45.0;	  ///< 法线夹角门控 °
	std::size_t minInliers = 40U;		  ///< 最少内点数
	int maxIterations = 5000;			  ///< RANSAC 迭代上限
	unsigned int fpfhNeighbors = 20U;	  ///< FPFH 邻域
	double modelDiagMm = 0.0;			  ///< 模型对角 mm；0=从点云计算
	double faceBandMm = 2.0;			  ///< 面带厚度 mm
	bool refineWithIcp = true;			  ///< 成功后点-面 ICP 精修
	bool skipTranslationCap = false;	  ///< 跳过平移幅度限制
};

/**
 * 体素下采样 → SPFH/FPFH → 互匹配+ratio → RANSAC → Kabsch → 可选 ICP
 * 用于扫描-模板世界系粗配；失败时 Host 可继续粗 ICP
 * @param sourceToTarget 输出刚体
 * @param inlierRatio 可选内点比例
 * @return false：特征不足、内点不够或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool
rigidRegisterFeatureRansac(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormalsNxNyNz,
						   const std::vector<float>& targetXyz, const std::vector<float>& targetNormalsNxNyNz,
						   Eigen::Isometry3d& sourceToTarget, double* inlierRatio, RigidRegisterRansacParams params,
						   std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONGLOBAL_H
