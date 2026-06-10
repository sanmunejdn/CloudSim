#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "MeshRemesh.h"
#include "VcgMeshTypes.h"

#include <algorithm>
#include <numeric>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>

namespace vcgalgo
{

bool isotropicRemesh(
	const std::vector<float>& triangleSoup,
	double targetEdgeLengthMm,
	std::vector<float>& outSoup,
	int iterations,
	std::string* errMsg)
{
	outSoup.clear();

	if (targetEdgeLengthMm <= 0.0)
	{
		if (errMsg) *errMsg = "target edge length must be positive";
		return false;
	}

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	if (iterations <= 0) iterations = 3;

	// 更新拓扑
	vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
	vcg::tri::UpdateTopology<VcgMesh>::VertexFace(mesh);
	vcg::tri::UpdateNormal<VcgMesh>::PerVertexNormalized(mesh);
	vcg::tri::UpdateFlags<VcgMesh>::FaceBorderFromFF(mesh);

	// 各向同性重网格
	vcg::tri::IsotropicRemeshing<VcgMesh>::Params rParams;
	rParams.SetTargetLen(static_cast<float>(targetEdgeLengthMm));
	rParams.SetFeatureAngleDeg(30.0f);
	rParams.iter = iterations;
	rParams.projectFlag = true;

	vcg::tri::IsotropicRemeshing<VcgMesh>::Do(mesh, rParams);

	internal::vcgMeshToSoup(mesh, outSoup);
	return !outSoup.empty();
}

} // namespace vcgalgo
