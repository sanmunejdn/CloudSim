#include "MeshRepair.h"
#include "VcgMeshTypes.h"

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/flag.h>

namespace vcgalgo
{

bool repairMesh(
	const std::vector<float>& triangleSoup,
	std::vector<float>& outSoup,
	const RepairParams& params,
	std::string* errMsg)
{
	outSoup.clear();

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	// 更新拓扑
	vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
	vcg::tri::UpdateTopology<VcgMesh>::VertexFace(mesh);
	vcg::tri::UpdateFlags<VcgMesh>::FaceBorderFromFF(mesh);

	// 去除重复顶点
	if (params.removeDuplicate)
	{
		vcg::tri::Clean<VcgMesh>::RemoveDuplicateVertex(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::VertexFace(mesh);
	}

	// 去除退化面
	if (params.removeDegenerate)
	{
		vcg::tri::Clean<VcgMesh>::RemoveZeroAreaFace(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
	}

	// 去除非流形
	if (params.removeNonManifold)
	{
		vcg::tri::Clean<VcgMesh>::RemoveNonManifoldFace(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
		vcg::tri::Clean<VcgMesh>::RemoveUnreferencedVertex(mesh);
	}

	// 填充孔洞
	if (params.fillHoles)
	{
		vcg::tri::UpdateFlags<VcgMesh>::FaceBorderFromFF(mesh);
		// EarCuttingFill 使用 TrivialEar 填充所有小于 maxSize 的孔洞
		vcg::tri::Hole<VcgMesh>::template EarCuttingFill<vcg::tri::TrivialEar<VcgMesh>>(
			mesh,
			params.holeMaxEdgeCount,
			false);
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
		vcg::tri::Clean<VcgMesh>::RemoveUnreferencedVertex(mesh);
	}

	internal::vcgMeshToSoup(mesh, outSoup);
	return !outSoup.empty();
}

} // namespace vcgalgo
