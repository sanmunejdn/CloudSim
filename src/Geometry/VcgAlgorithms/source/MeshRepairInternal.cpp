#include "MeshRepairInternal.h"

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/topology.h>

namespace vcgalgo::internal
{

namespace
{

int countActiveFaces(const VcgMesh& mesh)
{
	int count = 0;
	for (const auto& face : mesh.face)
	{
		if (!face.IsD())
		{
			++count;
		}
	}
	return count;
}

} // namespace

bool repairVcgMeshInPlace(VcgMesh& mesh, const RepairParams& params, RepairReport* report)
{
	vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
	vcg::tri::UpdateTopology<VcgMesh>::VertexFace(mesh);
	vcg::tri::UpdateFlags<VcgMesh>::FaceBorderFromFF(mesh);

	const int inputFaces = countActiveFaces(mesh);
	if (report != nullptr)
	{
		report->inputFaceCount = inputFaces;
	}

	if (params.removeDuplicate)
	{
		vcg::tri::Clean<VcgMesh>::RemoveDuplicateVertex(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::VertexFace(mesh);
	}

	if (params.removeDuplicateFaces)
	{
		const int before = countActiveFaces(mesh);
		vcg::tri::Clean<VcgMesh>::RemoveDuplicateFace(mesh);
		if (report != nullptr)
		{
			report->removedDuplicateFaces = before - countActiveFaces(mesh);
		}
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
	}

	if (params.removeDegenerate)
	{
		const int before = countActiveFaces(mesh);
		vcg::tri::Clean<VcgMesh>::RemoveZeroAreaFace(mesh);
		if (report != nullptr)
		{
			report->removedDegenerateFaces = before - countActiveFaces(mesh);
		}
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
	}

	if (params.removeNonManifold)
	{
		const int before = countActiveFaces(mesh);
		vcg::tri::Clean<VcgMesh>::RemoveNonManifoldFace(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
		vcg::tri::Clean<VcgMesh>::RemoveUnreferencedVertex(mesh);
		if (report != nullptr)
		{
			report->removedNonManifoldFaces = before - countActiveFaces(mesh);
		}
	}

	if (params.fillHoles)
	{
		const int before = countActiveFaces(mesh);
		vcg::tri::UpdateFlags<VcgMesh>::FaceBorderFromFF(mesh);
		vcg::tri::Hole<VcgMesh>::template EarCuttingFill<vcg::tri::TrivialEar<VcgMesh>>(
			mesh,
			params.holeMaxEdgeCount,
			false);
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
		vcg::tri::Clean<VcgMesh>::RemoveUnreferencedVertex(mesh);
		if (report != nullptr)
		{
			report->facesAddedByFill = countActiveFaces(mesh) - before;
		}
	}

	if (report != nullptr)
	{
		report->outputFaceCount = countActiveFaces(mesh);
	}
	return countActiveFaces(mesh) > 0;
}

} // namespace vcgalgo::internal
