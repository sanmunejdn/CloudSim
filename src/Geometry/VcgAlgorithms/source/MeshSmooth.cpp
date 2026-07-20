/// @file MeshSmooth.cpp
/// @brief MeshSmooth 实现

#include "MeshSmooth.h"

#include "MeshRepairInternal.h"
#include "VcgMeshTypes.h"

#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>

namespace vcgalgo
{
namespace
{
void markBoundaryVertices(VcgMesh& mesh)
{
	for (auto& vertex : mesh.vert)
	{
		vertex.ClearB();
	}
	vcg::tri::UpdateFlags<VcgMesh>::VertexBorderFromFaceBorder(mesh);
}

void runLaplacianOnMesh(VcgMesh& mesh, int iterations, bool cotangentWeight, bool preserveBoundary)
{
	if (iterations <= 0)
	{
		iterations = 3;
	}

	std::vector<std::pair<typename VcgMesh::VertexType*, typename VcgMesh::VertexType::CoordType>> borderPositions;
	if (preserveBoundary)
	{
		markBoundaryVertices(mesh);
		for (auto& vertex : mesh.vert)
		{
			if (!vertex.IsD() && vertex.IsB())
			{
				borderPositions.emplace_back(&vertex, vertex.cP());
			}
		}
	}

	vcg::tri::Smooth<VcgMesh>::VertexCoordLaplacian(mesh, iterations, false, cotangentWeight);

	if (preserveBoundary)
	{
		for (const auto& entry : borderPositions)
		{
			entry.first->P() = entry.second;
		}
	}
}

void runTaubinOnMesh(VcgMesh& mesh, int iterations, float lambda, float mu, bool preserveBoundary)
{
	if (iterations <= 0)
	{
		iterations = 3;
	}

	std::vector<std::pair<typename VcgMesh::VertexType*, typename VcgMesh::VertexType::CoordType>> borderPositions;
	if (preserveBoundary)
	{
		markBoundaryVertices(mesh);
		for (auto& vertex : mesh.vert)
		{
			if (!vertex.IsD() && vertex.IsB())
			{
				borderPositions.emplace_back(&vertex, vertex.cP());
			}
		}
	}

	vcg::tri::Smooth<VcgMesh>::VertexCoordTaubin(mesh, iterations, lambda, mu);

	if (preserveBoundary)
	{
		for (const auto& entry : borderPositions)
		{
			entry.first->P() = entry.second;
		}
	}
}

bool runSmoothOnMesh(VcgMesh& mesh, const MeshSmoothParams& params, std::string* errMsg)
{
	try
	{
		if (!internal::prepareMeshTopology(mesh, errMsg))
		{
			return false;
		}

		if (params.useTaubin)
		{
			double lambda = params.lambda;
			if (lambda <= 0.0)
			{
				lambda = 0.2;
			}
			if (lambda > 1.0)
			{
				lambda = 1.0;
			}
			const float lambdaF = static_cast<float>(lambda);
			const float mu = -lambdaF * 1.05f;
			runTaubinOnMesh(mesh, params.iterations, lambdaF, mu, params.preserveBoundary);
		}
		else
		{
			runLaplacianOnMesh(mesh, params.iterations, params.cotangentWeight, params.preserveBoundary);
		}

		vcg::tri::UpdateNormal<VcgMesh>::PerVertexNormalized(mesh);
		return true;
	}
	catch (const std::exception& e)
	{
		if (errMsg != nullptr)
		{
			*errMsg = e.what();
		}
		return false;
	}
}

} // namespace

bool applyMeshSmooth(const std::vector<float>& triangleSoup, std::vector<float>& outSoup,
					 const MeshSmoothParams& params, RepairReport* repairReport, std::string* errMsg)
{
	outSoup.clear();

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	if (params.repairBeforeSmooth)
	{
		if (!internal::repairVcgMeshInPlace(mesh, params.repairParams, repairReport))
		{
			if (errMsg != nullptr)
			{
				*errMsg = "applyMeshSmooth: repair before smooth failed";
			}
			return false;
		}
	}

	if (!runSmoothOnMesh(mesh, params, errMsg))
	{
		return false;
	}

	internal::vcgMeshToSoup(mesh, outSoup);
	return !outSoup.empty();
}

bool smoothLaplacian(const std::vector<float>& triangleSoup, int iterations, std::vector<float>& outSoup,
					 std::string* errMsg)
{
	MeshSmoothParams params;
	params.iterations = iterations;
	params.useTaubin = false;
	return applyMeshSmooth(triangleSoup, outSoup, params, nullptr, errMsg);
}

bool smoothTaubin(const std::vector<float>& triangleSoup, int iterations, double lambda, std::vector<float>& outSoup,
				  std::string* errMsg)
{
	MeshSmoothParams params;
	params.iterations = iterations;
	params.lambda = lambda;
	params.useTaubin = true;
	params.cotangentWeight = false;
	return applyMeshSmooth(triangleSoup, outSoup, params, nullptr, errMsg);
}

} // namespace vcgalgo
