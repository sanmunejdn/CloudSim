#ifndef POINTCLOUDALGORITHM_ADAPTIVEREMESH_H
#define POINTCLOUDALGORITHM_ADAPTIVEREMESH_H

/// @file AdaptiveRemesh.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 曲率自适应各向同性重网格（CGAL 5.5；无 Adaptive_sizing_field）

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{

struct AdaptiveRemeshParams
{
	double approxTolMm = 0.0;			 ///< ε；0 → 0.02 * characteristicEdgeMm
	double edgeMinMm = 0.0;				 ///< 0 → 0.25 * characteristicEdgeMm
	double edgeMaxMm = 0.0;				 ///< 0 → 2.0 * characteristicEdgeMm
	double characteristicEdgeMm = 0.0;	 ///< 特征边长 h（mm），须 >0 或由 edgeMin/Max 显式给出
	int refineIterations = 5;
	double featureAngleDeg = 30.0;
	int baseRemeshIterations = 3;
};

/**
 * 先均匀 remesh 到 Lmax，再按曲率边长场做有限次 split/collapse/flip/平滑
 * @return false：soup 非法、参数无效或 remesh 失败
 */
POINT_CLOUD_ALGORITHM_API bool adaptiveIsotropicRemesh(const std::vector<float>& triangleSoupIn,
													   std::vector<float>& triangleSoupOut,
													   const AdaptiveRemeshParams& params, std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_ADAPTIVEREMESH_H
