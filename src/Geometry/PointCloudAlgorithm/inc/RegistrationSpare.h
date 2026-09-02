#ifndef POINTCLOUDALGORITHM_REGISTRATIONSPARE_H
#define POINTCLOUDALGORITHM_REGISTRATIONSPARE_H

/// @file RegistrationSpare.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief SPARE 非刚性配准：对称点-面 + 变形图 + ARAP

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{
/// SPARE 求解参数（研究用途；详见 docs/spare_nonrigid_registration.md）
struct SpareRegisterParams
{
	double sampleRadiusRatio = 0.0;		 ///< 变形节点采样半径比；0=自动
	double wSmo = 0.01;					 ///< 平滑项权重
	double wRot = 1e-4;					 ///< 旋转正则
	double wArapCoarse = 500.0;			 ///< 粗阶段 ARAP 权重
	double wArapFine = 200.0;			 ///< 细阶段 ARAP 权重
	bool useSymmetricPointToPlane = true;
	bool useCoarseReg = true;
	bool useFineReg = true;
	bool normalizeScale = true;
	bool rigidPreAlign = false;			 ///< 前先点-面 ICP
	bool coarseGlobalAlign = false;
	double voxelPrefilterMm = 0.0;		 ///< 体素预滤波 mm；≤0 跳过
	int maxOuterIters = 30;
	double stopCoarse = 1e-3;
	double stopFine = 1e-4;
	std::size_t alignSampleCount = 3000U;
	int rigidPreAlignMaxIterations = 40;
	double rigidPreAlignMaxPairDistanceMm = 0.0;
	std::size_t rigidPreAlignMaxPoints = 4000U;
};

struct SpareRegisterResult
{
	/// 终态对应点平均点面距离（mm；normalize 时已反缩放）
	double meanErrorMm = 0.0;
	double meshScale = 1.0;
	int deformationNodeCount = 0;
};

/**
 * 点云→点云 SPARE；xyz/法线均为 3*N float（mm）
 * @return false：点数不足、法线缺失或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool
spareRegisterPointClouds(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormals,
						 const std::vector<float>& targetXyz, const std::vector<float>& targetNormals,
						 std::vector<float>& sourceXyzDeformedOut, std::vector<float>& sourceNormalsDeformedOut,
						 const SpareRegisterParams& params, SpareRegisterResult* stats, std::string* errMsg);

/**
 * 网格 soup（9*T）→ 点云目标
 * @return false：soup 非法或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool
spareRegisterMeshSoupToTarget(const std::vector<float>& sourceSoup, const std::vector<float>& targetXyz,
							  const std::vector<float>& targetNormals, std::vector<float>& sourceSoupDeformedOut,
							  const SpareRegisterParams& params, SpareRegisterResult* stats, std::string* errMsg);

/**
 * 网格 soup → 网格 soup
 * @return false：soup 非法或求解失败
 */
POINT_CLOUD_ALGORITHM_API bool spareRegisterMeshSoupToMeshSoup(const std::vector<float>& sourceSoup,
															   const std::vector<float>& targetSoup,
															   std::vector<float>& sourceSoupDeformedOut,
															   const SpareRegisterParams& params,
															   SpareRegisterResult* stats, std::string* errMsg);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONSPARE_H
