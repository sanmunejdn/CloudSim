/// @file MeshSimplify.cpp
/// @brief MeshSimplify 实现

#include "MeshSimplify.h"

#include "VcgMeshTypes.h"

#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <vcg/complex/algorithms/update/flag.h>

namespace vcgalgo
{
// 前向声明简化类（vcglib CRTP 模式要求）
class VcgEdgeCollapser;

// vcglib quadric edge collapse 需要自定义类继承 TriEdgeCollapseQuadric
class VcgEdgeCollapser
	: public vcg::tri::TriEdgeCollapseQuadric<VcgMesh, vcg::tri::BasicVertexPair<VcgMesh::VertexType>, VcgEdgeCollapser,
											  vcg::tri::QInfoStandard<VcgMesh::VertexType>>
{
public:
	using Base = vcg::tri::TriEdgeCollapseQuadric<VcgMesh, vcg::tri::BasicVertexPair<VcgMesh::VertexType>,
												  VcgEdgeCollapser, vcg::tri::QInfoStandard<VcgMesh::VertexType>>;
	using Base::Base;
};

bool simplifyQuadricEdgeCollapse(const std::vector<float>& triangleSoup, std::vector<float>& outSoup,
								 const SimplifyParams& params, std::string* errMsg)
{
	outSoup.clear();

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	// 目标面数
	int targetFaces = params.targetFaceCount;
	if (targetFaces <= 0)
	{
		targetFaces = static_cast<int>(mesh.face.size() / 2);
	}
	if (targetFaces <= 0)
	{
		if (errMsg)
			*errMsg = "too few faces to simplify";
		return false;
	}

	// 更新拓扑和边界标记
	vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
	vcg::tri::UpdateTopology<VcgMesh>::VertexFace(mesh);
	vcg::tri::UpdateFlags<VcgMesh>::FaceBorderFromFF(mesh);
	vcg::tri::UpdateFlags<VcgMesh>::VertexBorderFromFaceBorder(mesh);

	// 设置 quadric 简化参数
	vcg::tri::TriEdgeCollapseQuadricParameter qParams;
	qParams.QualityThr = params.qualityThreshold;
	qParams.PreserveBoundary = params.preserveBoundary;
	qParams.PreserveTopology = params.preserveTopology;
	qParams.QualityCheck = true;
	qParams.NormalCheck = false;
	qParams.OptimalPlacement = true;
	qParams.ScaleIndependent = true;

	// 执行简化
	vcg::LocalOptimization<VcgMesh> optimization(mesh, &qParams);
	optimization.Init<VcgEdgeCollapser>();
	optimization.SetTargetSimplices(static_cast<size_t>(targetFaces));
	optimization.DoOptimization();

	internal::vcgMeshToSoup(mesh, outSoup);

	if (outSoup.empty())
	{
		if (errMsg)
			*errMsg = "simplification produced empty mesh";
		return false;
	}

	return true;
}

} // namespace vcgalgo
