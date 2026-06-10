#include "MeshSmooth.h"
#include "VcgMeshTypes.h"

#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>

namespace vcgalgo
{

namespace
{

bool runSmoothPipeline(
	VcgMesh& mesh,
	int iterations,
	bool useTaubin,
	float lambda,
	float mu,
	std::string* errMsg)
{
	try
	{
		vcg::tri::UpdateTopology<VcgMesh>::FaceFace(mesh);
		vcg::tri::UpdateTopology<VcgMesh>::VertexFace(mesh);
		vcg::tri::UpdateFlags<VcgMesh>::FaceBorderFromFF(mesh);

		if (useTaubin)
		{
			vcg::tri::Smooth<VcgMesh>::VertexCoordTaubin(mesh, iterations, lambda, mu);
		}
		else
		{
			vcg::tri::Smooth<VcgMesh>::VertexCoordLaplacian(mesh, iterations);
		}

		vcg::tri::UpdateNormal<VcgMesh>::PerVertexNormalized(mesh);
		return true;
	}
	catch (const std::exception& e)
	{
		if (errMsg)
		{
			*errMsg = e.what();
		}
		return false;
	}
}

} // namespace

bool smoothLaplacian(
	const std::vector<float>& triangleSoup,
	int iterations,
	std::vector<float>& outSoup,
	std::string* errMsg)
{
	outSoup.clear();

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	if (iterations <= 0) iterations = 3;

	if (!runSmoothPipeline(mesh, iterations, false, 0.0f, 0.0f, errMsg))
	{
		return false;
	}

	internal::vcgMeshToSoup(mesh, outSoup);
	return !outSoup.empty();
}

bool smoothImplicitFairing(
	const std::vector<float>& triangleSoup,
	double lambda,
	std::vector<float>& outSoup,
	std::string* errMsg)
{
	outSoup.clear();

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	if (lambda <= 0.0) lambda = 0.2;
	if (lambda > 1.0) lambda = 1.0;

	const float mu = -1.0f * static_cast<float>(lambda) * 1.05f;
	const int steps = 3;
	if (!runSmoothPipeline(mesh, steps, true, static_cast<float>(lambda), mu, errMsg))
	{
		return false;
	}

	internal::vcgMeshToSoup(mesh, outSoup);
	return !outSoup.empty();
}

} // namespace vcgalgo
