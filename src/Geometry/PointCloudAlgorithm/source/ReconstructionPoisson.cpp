/// @file ReconstructionPoisson.cpp
/// @brief Poisson 隐式表面重建

#include "ReconstructionPoisson.h"

#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/poisson_surface_reconstruction.h>
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

bool buildPointNormalFromBuffers(const std::vector<float>& xyz, const std::vector<float>& normals,
								 std::vector<Point_with_normal>& points, std::string* errMsg)
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
		points.emplace_back(Point_3(xyz[b], xyz[b + 1U], xyz[b + 2U]),
							Vector_3(normals[b], normals[b + 1U], normals[b + 2U]));
	}
	return true;
}

void surfaceMeshToTriangleSoup(const Surface_mesh& mesh, std::vector<float>& triangleSoupOut)
{
	triangleSoupOut.clear();
	if (mesh.is_empty())
	{
		return;
	}

	const std::size_t estimatedTriangles = mesh.num_faces();
	triangleSoupOut.reserve(estimatedTriangles * 9U);

	for (const auto face : mesh.faces())
	{
		auto halfedge = mesh.halfedge(face);
		auto v0 = mesh.target(halfedge);
		auto v1 = mesh.target(mesh.next(halfedge));
		auto v2 = mesh.target(mesh.next(mesh.next(halfedge)));

		if (mesh.next(mesh.next(mesh.next(halfedge))) != halfedge)
		{
			continue;
		}

		const Point_3& p0 = mesh.point(v0);
		const Point_3& p1 = mesh.point(v1);
		const Point_3& p2 = mesh.point(v2);

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

bool reconstructPoisson(const std::vector<float>& xyz, const std::vector<float>& normalsNxNyNz,
						std::vector<float>& triangleSoupOut, double spacingMm, const double smAngleDeg,
						const double smRadiusRel, const double smDistanceRel, std::string* errMsg)
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
		points.begin(), points.end(), CGAL::First_of_pair_property_map<Point_with_normal>(),
		CGAL::Second_of_pair_property_map<Point_with_normal>(), mesh, spacingMm, smAngleDeg, smRadiusRel,
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

bool reconstructPoissonAuto(std::vector<float> xyz, std::vector<float>& triangleSoupOut, const double voxelPrefilterMm,
							const double outlierRemovalPercent, std::string* errMsg)
{
	std::vector<float> normals;
	if (!preprocessForReconstruction(xyz, normals, voxelPrefilterMm, outlierRemovalPercent, errMsg))
	{
		return false;
	}
	return reconstructPoisson(xyz, normals, triangleSoupOut, 0.0, 20.0, 30.0, 0.375, errMsg);
}

} // namespace pclalgo
