/// @file MeshBoolean.cpp
/// @brief MeshBoolean 实现

#include "GeoMeshBoolean.h"

#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <tuple>

#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>

namespace geoalgo
{
namespace
{
using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

void meshZExtent(const Mesh& mesh, double& zMin, double& zMax)
{
	zMin = std::numeric_limits<double>::max();
	zMax = std::numeric_limits<double>::lowest();
	for (const auto v : mesh.vertices())
	{
		const double z = mesh.point(v).z();
		zMin = std::min(zMin, z);
		zMax = std::max(zMax, z);
	}
}

void scaleMeshZAboutOrigin(Mesh& mesh, const double scale)
{
	if (scale <= 1.0)
	{
		return;
	}
	for (const auto v : mesh.vertices())
	{
		const Point_3 p = mesh.point(v);
		mesh.point(v) = Point_3(p.x(), p.y(), p.z() * scale);
	}
}

void scaleMeshXYAboutOrigin(Mesh& mesh, const double scale)
{
	if (scale <= 1.0)
	{
		return;
	}
	for (const auto v : mesh.vertices())
	{
		const Point_3 p = mesh.point(v);
		mesh.point(v) = Point_3(p.x() * scale, p.y() * scale, p.z());
	}
}

bool repairMeshForBoolean(Mesh& mesh, std::string* errMsg)
{
	namespace PMP = CGAL::Polygon_mesh_processing;
	PMP::stitch_borders(mesh);
	PMP::remove_degenerate_faces(mesh);
	if (mesh.number_of_faces() == 0)
	{
		if (errMsg)
		{
			*errMsg = "mesh has no faces after repair";
		}
		return false;
	}
	if (!CGAL::is_closed(mesh))
	{
		if (errMsg)
		{
			*errMsg = "mesh is not closed after soup conversion";
		}
		return false;
	}
	if (!PMP::does_bound_a_volume(mesh))
	{
		if (errMsg)
		{
			*errMsg = "mesh does not bound a volume";
		}
		return false;
	}
	PMP::orient_to_bound_a_volume(mesh);
	return true;
}

bool soupToMesh(const std::vector<float>& soup, Mesh& mesh, std::string* errMsg)
{
	namespace PMP = CGAL::Polygon_mesh_processing;
	if (soup.empty() || (soup.size() % 9U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "triangle soup must have 9 floats per triangle";
		}
		return false;
	}

	auto quantKey = [](const Point_3& p)
	{
		return std::make_tuple(static_cast<long long>(std::llround(p.x() / 1e-4)),
							   static_cast<long long>(std::llround(p.y() / 1e-4)),
							   static_cast<long long>(std::llround(p.z() / 1e-4)));
	};
	std::map<std::tuple<long long, long long, long long>, std::size_t> pointIndex;
	std::vector<Point_3> points;
	std::vector<std::vector<std::size_t>> polygons;
	points.reserve(soup.size() / 3U);
	polygons.reserve(soup.size() / 9U);

	auto vertexIndex = [&](const Point_3& p) -> std::size_t
	{
		const auto key = quantKey(p);
		const auto it = pointIndex.find(key);
		if (it != pointIndex.end())
		{
			return it->second;
		}
		const std::size_t idx = points.size();
		points.push_back(p);
		pointIndex.emplace(key, idx);
		return idx;
	};

	for (std::size_t i = 0; i + 8U < soup.size(); i += 9U)
	{
		const Point_3 p0(soup[i], soup[i + 1], soup[i + 2]);
		const Point_3 p1(soup[i + 3], soup[i + 4], soup[i + 5]);
		const Point_3 p2(soup[i + 6], soup[i + 7], soup[i + 8]);
		polygons.push_back({vertexIndex(p0), vertexIndex(p1), vertexIndex(p2)});
	}

	if (points.empty() || polygons.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty mesh from soup";
		}
		return false;
	}

	PMP::repair_polygon_soup(points, polygons);
	(void)PMP::orient_polygon_soup(points, polygons);
	mesh.clear();
	PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
	if (mesh.is_empty())
	{
		if (errMsg)
		{
			*errMsg = "polygon_soup_to_polygon_mesh produced empty mesh";
		}
		return false;
	}
	return repairMeshForBoolean(mesh, errMsg);
}

void meshToSoup(const Mesh& mesh, std::vector<float>& soup)
{
	soup.clear();
	for (const auto f : mesh.faces())
	{
		std::vector<Mesh::Vertex_index> verts;
		for (const auto v : vertices_around_face(mesh.halfedge(f), mesh))
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
		soup.push_back(static_cast<float>(p0.x()));
		soup.push_back(static_cast<float>(p0.y()));
		soup.push_back(static_cast<float>(p0.z()));
		soup.push_back(static_cast<float>(p1.x()));
		soup.push_back(static_cast<float>(p1.y()));
		soup.push_back(static_cast<float>(p1.z()));
		soup.push_back(static_cast<float>(p2.x()));
		soup.push_back(static_cast<float>(p2.y()));
		soup.push_back(static_cast<float>(p2.z()));
	}
}

void ensureToolClearsTargetZ(Mesh& tool, const Mesh& target)
{
	double tZ0 = 0.0;
	double tZ1 = 0.0;
	double cZ0 = 0.0;
	double cZ1 = 0.0;
	meshZExtent(target, tZ0, tZ1);
	meshZExtent(tool, cZ0, cZ1);
	const double targetH = tZ1 - tZ0;
	const double toolH = cZ1 - cZ0;
	if (targetH <= 0.0 || toolH <= 0.0)
	{
		return;
	}
	const double minToolH = targetH * 1.05 + 2.0;
	if (toolH >= minToolH)
	{
		return;
	}
	scaleMeshZAboutOrigin(tool, minToolH / toolH);
}

bool runBoolean(Mesh& target, Mesh& tool, const MeshBooleanOp op, Mesh& out, std::string* errMsg)
{
	namespace PMP = CGAL::Polygon_mesh_processing;
	const auto params = CGAL::parameters::throw_on_self_intersection(false);
	bool ok = false;
	switch (op)
	{
	case MeshBooleanOp::Difference:
		ok = PMP::corefine_and_compute_difference(target, tool, out, params, params, params);
		break;
	case MeshBooleanOp::Union:
		ok = PMP::corefine_and_compute_union(target, tool, out, params, params, params);
		break;
	case MeshBooleanOp::Intersection:
		ok = PMP::corefine_and_compute_intersection(target, tool, out, params, params, params);
		break;
	}
	if (!ok && errMsg)
	{
		std::ostringstream oss;
		oss << "CGAL boolean failed";
		*errMsg = oss.str();
	}
	return ok;
}

std::vector<float> makeBoxSoup(const double lx, const double ly, const double lz)
{
	const double hx = lx * 0.5;
	const double hy = ly * 0.5;
	const double hz = lz * 0.5;
	std::vector<float> soup;
	auto tri = [&](double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz)
	{
		soup.push_back(static_cast<float>(ax));
		soup.push_back(static_cast<float>(ay));
		soup.push_back(static_cast<float>(az));
		soup.push_back(static_cast<float>(bx));
		soup.push_back(static_cast<float>(by));
		soup.push_back(static_cast<float>(bz));
		soup.push_back(static_cast<float>(cx));
		soup.push_back(static_cast<float>(cy));
		soup.push_back(static_cast<float>(cz));
	};
	tri(-hx, -hy, -hz, hx, -hy, -hz, hx, hy, -hz);
	tri(-hx, -hy, -hz, hx, hy, -hz, -hx, hy, -hz);
	tri(-hx, -hy, hz, hx, hy, hz, hx, -hy, hz);
	tri(-hx, -hy, hz, -hx, hy, hz, hx, hy, hz);
	tri(-hx, -hy, -hz, -hx, hy, -hz, -hx, hy, hz);
	tri(-hx, -hy, -hz, -hx, hy, hz, -hx, -hy, hz);
	tri(hx, -hy, -hz, hx, -hy, hz, hx, hy, hz);
	tri(hx, -hy, -hz, hx, hy, hz, hx, hy, -hz);
	tri(-hx, -hy, -hz, -hx, -hy, hz, hx, -hy, hz);
	tri(-hx, -hy, -hz, hx, -hy, hz, hx, -hy, -hz);
	tri(-hx, hy, -hz, hx, hy, hz, -hx, hy, hz);
	tri(-hx, hy, -hz, hx, hy, -hz, hx, hy, hz);
	return soup;
}

} // namespace

bool meshBooleanCompute(const std::vector<float>& targetSoup, const std::vector<float>& toolSoup,
						const MeshBooleanOp op, std::vector<float>& outSoup, std::string* errMsg)
{
	outSoup.clear();
	Mesh target;
	Mesh tool;
	if (!soupToMesh(targetSoup, target, errMsg))
	{
		return false;
	}
	if (!soupToMesh(toolSoup, tool, errMsg))
	{
		return false;
	}
	if (op == MeshBooleanOp::Difference)
	{
		ensureToolClearsTargetZ(tool, target);
		scaleMeshXYAboutOrigin(tool, 1.001);
	}
	Mesh result;
	if (!runBoolean(target, tool, op, result, errMsg))
	{
		if (op != MeshBooleanOp::Difference)
		{
			return false;
		}
		Mesh toolRetry = tool;
		scaleMeshXYAboutOrigin(toolRetry, 1.003);
		ensureToolClearsTargetZ(toolRetry, target);
		if (!runBoolean(target, toolRetry, op, result, errMsg))
		{
			return false;
		}
	}
	meshToSoup(result, outSoup);
	if (outSoup.empty())
	{
		if (errMsg)
		{
			*errMsg = "boolean produced empty soup";
		}
		return false;
	}
	return true;
}

bool meshBooleanRunSelfTest(std::string* errMsg)
{
	const std::vector<float> box = makeBoxSoup(100.0, 100.0, 100.0);
	const std::vector<float> tool = makeBoxSoup(50.0, 50.0, 120.0);
	std::vector<float> out;
	return meshBooleanCompute(box, tool, MeshBooleanOp::Difference, out, errMsg) && out.size() >= 9U;
}

} // namespace geoalgo
