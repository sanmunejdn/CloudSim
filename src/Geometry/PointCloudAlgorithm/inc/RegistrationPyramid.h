#ifndef POINTCLOUDALGORITHM_REGISTRATIONPYRAMID_H
#define POINTCLOUDALGORITHM_REGISTRATIONPYRAMID_H

/// @file RegistrationPyramid.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 几何多分辨率金字塔编排：分层 remesh + 位移 prolongate + 调用现有 SDF/SPARE（不改求解器）

#include "point_cloud_algorithm_global.h"

#include "RegistrationSdf.h"
#include "RegistrationSpare.h"

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{

enum class PyramidSolver : int
{
	Sdf = 0,
	Spare = 1,
};

struct PyramidRegisterParams
{
	double baseEdgeLengthMm = 0.0; ///< h（mm）；0=取源中位边长
	int layers = 3;				   ///< 固定 3：L0=4h / L1=2h / L2=h
	double layerScale = 2.0;
	bool rigidPreAlign = true; ///< 仅最粗层交给求解器做 ICP
	bool useFineRegOnLastLayer = false;
	PyramidSolver solver = PyramidSolver::Sdf;
	SdfRegisterParams sdf;
	SpareRegisterParams spare;
	int remeshIterations = 3;
};

struct PyramidRegisterResult
{
	/// 末层求解器平均点面误差（mm）
	double meanErrorMm = 0.0;
	int deformationNodeCount = 0;
	double baseEdgeLengthMmUsed = 0.0;
	int layersRun = 0;
	std::string debugSummary;
};

/**
 * 网格 soup → 网格 soup 几何金字塔；每层从原始几何 remesh，层间 NN 传递位移；输出为细层 remesh 拓扑
 * @return false：soup 非法、remesh/prolongate/求解失败
 */
POINT_CLOUD_ALGORITHM_API bool
pyramidRegisterMeshSoupToMeshSoup(const std::vector<float>& sourceSoup, const std::vector<float>& targetSoup,
								  std::vector<float>& sourceSoupDeformedOut, const PyramidRegisterParams& params,
								  PyramidRegisterResult* stats, std::string* errMsg);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONPYRAMID_H
