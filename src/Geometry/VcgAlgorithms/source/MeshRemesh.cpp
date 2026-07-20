#ifndef NOMINMAX
#define NOMINMAX

/// @file MeshRemesh.cpp
/// @brief MeshRemesh 实现

#endif

#include "MeshRemesh.h"

#include "MeshRepairInternal.h"
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

#if defined(_MSC_VER)
#include <windows.h>
#endif

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

// 仅防止目标远小于当前中位导致一次性爆炸；不上抬目标
double clampTargetForRemeshSafety(const double targetMm, const double medianMm)
{
	if (!(medianMm > 1e-9) || !(targetMm > 0.0))
	{
		return targetMm;
	}
	const double minAllowed = medianMm * 0.05;
	if (targetMm < minAllowed)
	{
		return minAllowed;
	}
	return targetMm;
}

double medianEdgeLengthOfMesh(const VcgMesh& mesh)
{
	std::vector<double> lens;
	lens.reserve(static_cast<std::size_t>(mesh.FN()) * 3U);
	for (const auto& f : mesh.face)
	{
		if (f.IsD())
		{
			continue;
		}
		for (int e = 0; e < 3; ++e)
		{
			const double len = (f.P0(e) - f.P1(e)).Norm();
			if (len > 1e-12)
			{
				lens.push_back(len);
			}
		}
	}
	if (lens.empty())
	{
		return 0.0;
	}
	std::nth_element(lens.begin(), lens.begin() + static_cast<std::ptrdiff_t>(lens.size() / 2U), lens.end());
	return lens[lens.size() / 2U];
}

#if defined(_MSC_VER)
bool remeshDoProtected(VcgMesh& mesh, vcg::tri::IsotropicRemeshing<VcgMesh>::Params& params)
{
	__try
	{
		vcg::tri::IsotropicRemeshing<VcgMesh>::Do(mesh, params);
		return true;
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		return false;
	}
}
#else
bool remeshDoProtected(VcgMesh& mesh, vcg::tri::IsotropicRemeshing<VcgMesh>::Params& params)
{
	vcg::tri::IsotropicRemeshing<VcgMesh>::Do(mesh, params);
	return true;
}
#endif

} // namespace

bool computeMedianEdgeLengthMm(const std::vector<float>& triangleSoup, double& outMedianMm, std::string* errMsg)
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

bool isotropicRemesh(const std::vector<float>& triangleSoup, double targetEdgeLengthMm, std::vector<float>& outSoup,
					 int iterations, double featureAngleDeg, std::string* errMsg)
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
		RepairParams repairParams;
		repairParams.fillHoles = false;
		if (!internal::repairVcgMeshInPlace(mesh, repairParams, nullptr))
		{
			if (errMsg)
			{
				*errMsg = "isotropic remesh: repair left no faces";
			}
			return false;
		}

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

		const double medianEdge = medianEdgeLengthOfMesh(mesh);
		const double clampedTarget = clampTargetForRemeshSafety(targetEdgeLengthMm, medianEdge);

		vcg::tri::UpdateBounding<VcgMesh>::Box(mesh);
		const float bboxDiag = static_cast<float>(mesh.bbox.Diag());

		vcg::tri::IsotropicRemeshing<VcgMesh>::Params rParams;
		rParams.SetTargetLen(static_cast<float>(clampedTarget));
		rParams.SetFeatureAngleDeg(static_cast<float>(featureAngleDeg));
		rParams.iter = iterations;
		// CAD soup 投影回自身易把拓扑打坏
		rParams.projectFlag = false;
		rParams.cleanFlag = true;
		rParams.userSelectedCreases = false;
		rParams.maxSurfDist = std::max(1e-3f, bboxDiag * 0.001f);
		rParams.surfDistCheck = false;

		if (!remeshDoProtected(mesh, rParams))
		{
			if (errMsg)
			{
				*errMsg = "isotropic remesh crashed or aborted internally";
			}
			return false;
		}

		vcg::tri::Clean<VcgMesh>::RemoveUnreferencedVertex(mesh);
		vcg::tri::Allocator<VcgMesh>::CompactEveryVector(mesh);
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
