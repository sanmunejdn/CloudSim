#ifndef POINTCLOUDALGORITHM_SDF_NODESAMPLER_H
#define POINTCLOUDALGORITHM_SDF_NODESAMPLER_H

/// @file SdfNodeSampler.h
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
	std::vector<Eigen::Vector3d> nodeRest;			 ///< 节点静止位置
	std::vector<std::vector<int>> nodeNeighbors;	 ///< 节点邻接（kNN）
	std::vector<std::vector<SkinWeight>> vertSkin; ///< 每顶点蒙皮
};

/// sampleRadiusMm：节点间距；kSkin：每点影响节点数
bool buildDeformGraph(const std::vector<float>& xyz, double sampleRadiusMm, int kSkin, int kNodeNeighbors,
					  DeformGraph& out, std::string* errMsg);

} // namespace sdf
} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SDF_NODESAMPLER_H
