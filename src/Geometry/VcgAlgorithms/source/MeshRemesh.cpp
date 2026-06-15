#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "MeshRemesh.h"
#include "VcgMeshTypes.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/exception.h>

namespace vcgalgo
{
namespace
{

double edgeLengthMm(const float ax, const float ay, const float az, const float bx, const float by, const float bz)
{
	const double dx = static_cast<double>(bx - ax);
	const double dy = static_cast<double>(by - ay);
	const double dz = static_cast<double>(bz - az);
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

bool computeMedianEdgeLengthMm(
	const std::vector<float>& triangleSoup,
	double& outMedianMm,
	std::string* errMsg)
{
	outMedianMm = 0.0;
	if (triangleSoup.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "triangle soup too small for edge length statistics";
		}
		return false;
	}

	std::vector<double> edgeLengths;
	edgeLengths.reserve(triangleSoup.size());
	for (std::size_t t = 0U; t + 8U < triangleSoup.size(); t += 9U)
	{
		const float* v0 = &triangleSoup[t];
		const float* v1 = &triangleSoup[t + 3U];
		const float* v2 = &triangleSoup[t + 6U];
		edgeLengths.push_back(edgeLengthMm(v0[0], v0[1], v0[2], v1[0], v1[1], v1[2]));
		edgeLengths.push_back(edgeLengthMm(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2]));
		edgeLengths.push_back(edgeLengthMm(v2[0], v2[1], v2[2], v0[0], v0[1], v0[2]));
	}
	if (edgeLengths.empty())
	{
		if (errMsg)
		{
			*errMsg = "no edges for median edge length";
		}
		return false;
	}

	std::sort(edgeLengths.begin(), edgeLengths.end());
	outMedianMm = edgeLengths[edgeLengths.size() / 2U];
	return outMedianMm > 1e-9;
}

bool isotropicRemesh(
	const std::vector<float>& triangleSoup,
	double targetEdgeLengthMm,
	std::vector<float>& outSoup,
	int iterations,
	double featureAngleDeg,
	std::string* errMsg)
{
	outSoup.clear();

	if (targetEdgeLengthMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "target edge length must be positive";
		}
		return false;
	}

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	if (iterations <= 0)
	{
		iterations = 3;
	}
	if (featureAngleDeg <= 0.0)
	{
		featureAngleDeg = 30.0;
	}

	try
	{
		vcg::tri::Clean<VcgMesh>::RemoveUnreferencedVertex(mesh);
		vcg::tri::Allocator<VcgMesh>::CompactEveryVector(mesh);
		if (mesh.FN() < 1)
		{
			if (errMsg)
			{
				*errMsg = "isotropic remesh: no faces after cleanup";
			}
			return false;
		}

		if (!internal::prepareMeshTopology(mesh, errMsg))
		{
			return false;
		}

		vcg::tri::UpdateBounding<VcgMesh>::Box(mesh);
		const float bboxDiag = static_cast<float>(mesh.bbox.Diag());

		vcg::tri::IsotropicRemeshing<VcgMesh>::Params rParams;
		rParams.SetTargetLen(static_cast<float>(targetEdgeLengthMm));
		rParams.SetFeatureAngleDeg(static_cast<float>(featureAngleDeg));
		rParams.iter = iterations;
		rParams.projectFlag = true;
		rParams.cleanFlag = true;
		rParams.userSelectedCreases = false;
		rParams.maxSurfDist = std::max(1e-3f, bboxDiag * 0.001f);
		rParams.surfDistCheck = true;

		vcg::tri::IsotropicRemeshing<VcgMesh>::Do(mesh, rParams);

		internal::vcgMeshToSoup(mesh, outSoup);
	}
	catch (const vcg::MissingComponentException&)
	{
		if (errMsg)
		{
			*errMsg = "isotropic remesh: mesh missing required VCG component";
		}
		return false;
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = std::string("isotropic remesh failed: ") + ex.what();
		}
		return false;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "isotropic remesh failed with internal error";
		}
		return false;
	}

	if (outSoup.empty())
	{
		if (errMsg)
		{
			*errMsg = "isotropic remesh produced empty output";
		}
		return false;
	}
	return true;
}

} // namespace vcgalgo
