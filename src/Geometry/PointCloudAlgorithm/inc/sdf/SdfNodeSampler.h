#ifndef POINTCLOUDALGORITHM_SDF_NODESAMPLER_H
#define POINTCLOUDALGORITHM_SDF_NODESAMPLER_H

/// @file SdfNodeSampler.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief FPS 变形节点采样与蒙皮权重

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace pclalgo
{
namespace sdf
{

struct SkinWeight
{
	int node = -1;
	double w = 0.0;
};

struct DeformGraph
{
	std::vector<Eigen::Vector3d> nodeRest;
	std::vector<std::vector<int>> nodeNeighbors;
	std::vector<std::vector<SkinWeight>> vertSkin;
};

/// sampleRadiusMm：节点间距；kSkin：每点影响节点数
/// vertAdj 非空时沿网格邻接蒙皮/连边，避免凹槽两侧欧氏 kNN 跨空腔拉丝
bool buildDeformGraph(const std::vector<float>& xyz, double sampleRadiusMm, int kSkin, int kNodeNeighbors,
					  DeformGraph& out, std::string* errMsg,
					  const std::vector<std::vector<int>>* vertAdj = nullptr);

} // namespace sdf
} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SDF_NODESAMPLER_H
