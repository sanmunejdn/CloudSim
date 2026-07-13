#include "MeshSurfaceReconstructionPartitionCgal.h"
#include "MeshSurfaceReconstructionPartitionCommon.h"

#include "RunLogger.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

using CgalKernel = CGAL::Simple_cartesian<double>;
using CgalPoint = CgalKernel::Point_3;
using CgalMesh = CGAL::Surface_mesh<CgalPoint>;
using FaceDescriptor = CgalMesh::Face_index;

bool buildCgalMesh(const IndexedMeshLite& mesh, CgalMesh& outMesh)
{
	outMesh.clear();
	const int faceCount = static_cast<int>(mesh.faces.size() / 3U);
	if (faceCount < 4)
	{
		return false;
	}

	std::unordered_map<int, CgalMesh::Vertex_index> globalToVertex;
	for (int fi = 0; fi < faceCount; ++fi)
	{
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		std::array<CgalMesh::Vertex_index, 3> tri{};
		for (int c = 0; c < 3; ++c)
		{
			const int gv = mesh.faces[b + static_cast<std::size_t>(c)];
			auto it = globalToVertex.find(gv);
			if (it == globalToVertex.end())
			{
				const std::size_t vb = static_cast<std::size_t>(gv) * 3U;
				if (vb + 2U >= mesh.vertices.size())
				{
					return false;
				}
				const CgalMesh::Vertex_index vi = outMesh.add_vertex(CgalPoint(
					mesh.vertices[vb],
					mesh.vertices[vb + 1U],
					mesh.vertices[vb + 2U]));
				globalToVertex[gv] = vi;
				tri[static_cast<std::size_t>(c)] = vi;
			}
			else
			{
				tri[static_cast<std::size_t>(c)] = it->second;
			}
		}
		outMesh.add_face(tri[0], tri[1], tri[2]);
	}
	return static_cast<int>(outMesh.number_of_faces()) == faceCount;
}

} // namespace

bool collectSdfSegmentSeedFaces(
	const IndexedMeshLite& mesh,
	const int segmentCount,
	std::vector<int>& outSeedFaceIndices,
	std::string* errMsg)
{
	outSeedFaceIndices.clear();
	const int faceCount = static_cast<int>(mesh.faces.size() / 3U);
	if (faceCount < 12 || segmentCount < 2)
	{
		return false;
	}
	if (faceCount > 80000)
	{
		return false;
	}

	try
	{
		CgalMesh sm;
		if (!buildCgalMesh(mesh, sm))
		{
			if (errMsg)
			{
				*errMsg = "cgal mesh build failed for sdf";
			}
			return false;
		}

		auto sdfPmap = sm.add_property_map<FaceDescriptor, double>("f:sdf", 0.0).first;
		(void)CGAL::sdf_values(
			sm,
			sdfPmap,
			2.0 / 3.0 * CGAL_PI,
			15,
			true,
			CGAL::get(boost::vertex_point, sm),
			CgalKernel());

		const std::size_t clusters = static_cast<std::size_t>(
			std::max(2, std::min(segmentCount, faceCount / 8)));
		auto segPmap = sm.add_property_map<FaceDescriptor, std::size_t>("f:seg", std::size_t(0)).first;
		(void)CGAL::segmentation_from_sdf_values(
			sm,
			sdfPmap,
			segPmap,
			clusters,
			0.26,
			false,
			CGAL::get(boost::vertex_point, sm),
			CgalKernel());

		std::unordered_map<std::size_t, std::pair<double, int>> bestPerSegment;
		int fi = 0;
		for (FaceDescriptor fd : sm.faces())
		{
			const std::size_t label = segPmap[fd];
			const double sdf = sdfPmap[fd];
			auto& best = bestPerSegment[label];
			if (sdf > best.first)
			{
				best = {sdf, fi};
			}
			++fi;
		}

		for (const auto& kv : bestPerSegment)
		{
			if (kv.second.second >= 0)
			{
				outSeedFaceIndices.push_back(kv.second.second);
			}
		}
		return !outSeedFaceIndices.empty();
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "cgal sdf segmentation failed";
		}
		return false;
	}
}

bool partitionQuadDomainsCgalChartHybrid(
	const IndexedMeshLite& mesh,
	const MeshSurfaceReconstructParams& params,
	std::vector<QuadPatch>& patches,
	int& outJunctionCount,
	MeshSurfaceReconstructReport* partitionStats,
	std::string* errMsg)
{
	MeshSurfaceReconstructParams hybridParams = params;
	hybridParams.partitionMode = MeshSurfacePartitionMode::HybridNormalCvt;
	if (!partitionQuadDomainsHybrid(mesh, hybridParams, patches, outJunctionCount, partitionStats, errMsg))
	{
		return false;
	}
	assignAllPatchCornerMetadata(mesh, patches);
	RunLogger::info(
		std::string("cgal chart hybrid: ") + std::to_string(patches.size()) + " patches");
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
