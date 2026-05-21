#include "Reconstruction.h"

#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/property_map.h>

namespace pclalgo
{

namespace
{

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Point_with_normal = std::pair<Point_3, Vector_3>;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

bool buildPointNormalFromBuffers(
	const std::vector<float>& xyz,
	const std::vector<float>& normals,
	std::vector<Point_with_normal>& points,
	std::string* errMsg)
{
	if (!validXyzLength(xyz) || normals.size() != xyz.size())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "xyz/normals size mismatch";
		}
		return false;
	}
	points.clear();
	points.reserve(pointCountFromXyz(xyz));
	for (std::size_t i = 0; i < pointCountFromXyz(xyz); ++i)
	{
		const std::size_t b = i * 3U;
		points.emplace_back(
			Point_3(xyz[b], xyz[b + 1U], xyz[b + 2U]),
			Vector_3(normals[b], normals[b + 1U], normals[b + 2U]));
	}
	return true;
}

void surfaceMeshToTriangleSoup(const Surface_mesh& mesh, std::vector<float>& triangleSoupOut)
{
	triangleSoupOut.clear();
	for (const auto face : mesh.faces())
	{
		std::vector<Surface_mesh::Vertex_index> verts;
		for (const auto v : vertices_around_face(mesh.halfedge(face), mesh))
		{
			verts.push_back(v);
		}
		if (verts.size() < 3U)
		{
			continue;
		}
		const Point_3& p0 = mesh.point(verts[0]);
		const Point_3& p1 = mesh.point(verts[1]);
		const Point_3& p2 = mesh.point(verts[2]);
		triangleSoupOut.push_back(static_cast<float>(p0.x()));
		triangleSoupOut.push_back(static_cast<float>(p0.y()));
		triangleSoupOut.push_back(static_cast<float>(p0.z()));
		triangleSoupOut.push_back(static_cast<float>(p1.x()));
		triangleSoupOut.push_back(static_cast<float>(p1.y()));
		triangleSoupOut.push_back(static_cast<float>(p1.z()));
		triangleSoupOut.push_back(static_cast<float>(p2.x()));
		triangleSoupOut.push_back(static_cast<float>(p2.y()));
		triangleSoupOut.push_back(static_cast<float>(p2.z()));
	}
}

} // namespace

bool reconstructPoisson(
	const std::vector<float>& xyz,
	const std::vector<float>& normalsNxNyNz,
	std::vector<float>& triangleSoupOut,
	double spacingMm,
	const double smAngleDeg,
	const double smRadiusRel,
	const double smDistanceRel,
	std::string* errMsg)
{
	triangleSoupOut.clear();

	std::vector<Point_with_normal> points;
	if (!buildPointNormalFromBuffers(xyz, normalsNxNyNz, points, errMsg))
	{
		return false;
	}
	if (points.size() < 3U)
	{
		if (errMsg != nullptr)
		{
			*errMsg = "too few points for Poisson";
		}
		return false;
	}

	if (spacingMm <= 0.0)
	{
		spacingMm = computeAverageSpacingMm(xyz, 6);
		if (spacingMm <= 0.0)
		{
			spacingMm = 1.0;
		}
	}

	Surface_mesh mesh;
	const bool ok = CGAL::poisson_surface_reconstruction_delaunay(
		points.begin(),
		points.end(),
		CGAL::First_of_pair_property_map<Point_with_normal>(),
		CGAL::Second_of_pair_property_map<Point_with_normal>(),
		mesh,
		spacingMm,
		smAngleDeg,
		smRadiusRel,
		smDistanceRel);

	if (!ok || mesh.is_empty())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "Poisson surface reconstruction failed";
		}
		return false;
	}

	surfaceMeshToTriangleSoup(mesh, triangleSoupOut);
	return !triangleSoupOut.empty();
}

bool reconstructScaleSpace(
	const std::vector<float>& xyz,
	std::vector<float>& triangleSoupOut,
	const std::size_t smoothIterations,
	double meshingRadiusMm,
	std::string* errMsg)
{
	triangleSoupOut.clear();
	if (!validXyzLength(xyz) || xyz.empty())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "invalid xyz";
		}
		return false;
	}

	std::vector<Point_3> points;
	points.reserve(pointCountFromXyz(xyz));
	for (std::size_t i = 0; i < pointCountFromXyz(xyz); ++i)
	{
		const std::size_t b = i * 3U;
		points.emplace_back(xyz[b], xyz[b + 1U], xyz[b + 2U]);
	}

	CGAL::Scale_space_surface_reconstruction_3<Kernel> recon(points.begin(), points.end());
	recon.increase_scale(smoothIterations);
	recon.reconstruct_surface();

	if (meshingRadiusMm <= 0.0)
	{
		const Eigen::AlignedBox3d box = computeBoundingBox(xyz);
		meshingRadiusMm = box.diagonal().norm() * 0.05;
		if (meshingRadiusMm <= 0.0)
		{
			meshingRadiusMm = 1.0;
		}
	}

	const std::vector<Point_3> ssPoints(recon.points_begin(), recon.points_end());
	for (auto fit = recon.facets_begin(); fit != recon.facets_end(); ++fit)
	{
		const Point_3& p0 = ssPoints[(*fit)[0]];
		const Point_3& p1 = ssPoints[(*fit)[1]];
		const Point_3& p2 = ssPoints[(*fit)[2]];
		triangleSoupOut.push_back(static_cast<float>(p0.x()));
		triangleSoupOut.push_back(static_cast<float>(p0.y()));
		triangleSoupOut.push_back(static_cast<float>(p0.z()));
		triangleSoupOut.push_back(static_cast<float>(p1.x()));
		triangleSoupOut.push_back(static_cast<float>(p1.y()));
		triangleSoupOut.push_back(static_cast<float>(p1.z()));
		triangleSoupOut.push_back(static_cast<float>(p2.x()));
		triangleSoupOut.push_back(static_cast<float>(p2.y()));
		triangleSoupOut.push_back(static_cast<float>(p2.z()));
	}

	if (triangleSoupOut.empty())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "Scale-space reconstruction produced no facets";
		}
		return false;
	}
	(void)meshingRadiusMm;
	return true;
}

bool reconstructPoissonAuto(
	std::vector<float> xyz,
	std::vector<float>& triangleSoupOut,
	const double voxelPrefilterMm,
	const double outlierRemovalPercent,
	std::string* errMsg)
{
	std::vector<float> normals;
	if (!preprocessForReconstruction(xyz, normals, voxelPrefilterMm, outlierRemovalPercent, errMsg))
	{
		return false;
	}
	return reconstructPoisson(xyz, normals, triangleSoupOut, 0.0, 20.0, 30.0, 0.375, errMsg);
}

} // namespace pclalgo
