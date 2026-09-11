#ifndef POINTCLOUDALGORITHM_REGISTRATIONGLOBALPCL_H
#define POINTCLOUDALGORITHM_REGISTRATIONGLOBALPCL_H

/// @file RegistrationGlobalPcl.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PCL FPFH + SAC 全局粗配（可选；需 CLOUDSIM_HAS_PCL）

#include "point_cloud_algorithm_global.h"
#include "RegistrationGlobal.h"

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
/// PCL 全局粗配参数；距离类 ≤0 时按包围盒对角自动
struct PclGlobalAlignParams
{
	double featureVoxelMm = 0.0;	 ///< 体素 mm；0=自动（对角×1.5%）
	double normalRadiusMm = 0.0;	 ///< 法线半径；0=2.5×体素
	double fpfhRadiusMm = 0.0;		 ///< FPFH 半径；0=5×体素
	double inlierDistanceMm = 0.0;	 ///< 对应距离 mm；0=对角×2.5%
	double similarityThreshold = 0.92;
	float inlierFraction = 0.25f; ///< SAC 搜索门槛
	float minAcceptInlierRatio = 0.45f; ///< 正向内点比；过低易接受错转角
	float minAcceptReverseInlierRatio = 0.35f; ///< 反向（目标→源）内点比
	double maxAcceptMeanNnFactor = 1.2; ///< 全体点均 NN ≤ inlierDistance×此系数
	double icpMaxNormalAngleDeg = 45.0; ///< ICP 精修法线门 °；≤0 关门
	int maxIterations = 50000;
	int numberOfSamples = 3;
	int correspondenceRandomness = 15;
	unsigned int randomSeed = 42U; ///< 固定种子，避免每次结果漂移
	bool refineWithIcp = true;
	bool tryReverse = true; ///< 反向 SAC 择优，抑制翻面/错平面
	std::size_t maxFeaturePoints = 4000U;
};

/**
 * PCL VoxelGrid → 强制重估法线 → FPFH → SampleConsensusPrerejective → 可选点-面 ICP
 * 无 CLOUDSIM_HAS_PCL 时恒返回 false
 */
POINT_CLOUD_ALGORITHM_API bool
rigidRegisterFeatureRansacPcl(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormalsNxNyNz,
							  const std::vector<float>& targetXyz, const std::vector<float>& targetNormalsNxNyNz,
							  Eigen::Isometry3d& sourceToTarget, double* inlierRatio, PclGlobalAlignParams params,
							  std::string* errMsg = nullptr);

/// 从自研 RANSAC 参数填一份合理默认（供 SPARE / 模板粗配共用）
POINT_CLOUD_ALGORITHM_API PclGlobalAlignParams pclParamsFromRigidRansac(const RigidRegisterRansacParams& src);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONGLOBALPCL_H
