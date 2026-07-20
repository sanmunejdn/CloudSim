/// @file Reconstruction.cpp
/// @brief Reconstruction 实现

#include "Reconstruction.h"

#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"

#include <algorithm>
#include <map>
#include <tuple>

#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
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

	// 预分配内存：假设大部分面是三角形
	const std::size_t estimatedTriangles = mesh.num_faces();
	triangleSoupOut.reserve(estimatedTriangles * 9U);

	for (const auto face : mesh.faces())
	{
		// 直接访问顶点，避免创建临时vector
		auto halfedge = mesh.halfedge(face);
		auto v0 = mesh.target(halfedge);
		auto v1 = mesh.target(mesh.next(halfedge));
		auto v2 = mesh.target(mesh.next(mesh.next(halfedge)));

		// 检查是否为三角形（通过检查next(next(next(h)))是否回到h）
		if (mesh.next(mesh.next(mesh.next(halfedge))) != halfedge)
		{
			// 非三角形面，跳过或处理
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

void pushTriToSoup(std::vector<float>& soup, const Point_3& p0, const Point_3& p1, const Point_3& p2)
{
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

void buildSoupFromPolygons(const std::vector<Point_3>& points, const std::vector<std::vector<std::size_t>>& polygons,
						   std::vector<float>& soup)
{
	soup.clear();
	soup.reserve(polygons.size() * 9U);
	for (const auto& poly : polygons)
	{
		if (poly.size() < 3U)
		{
			continue;
		}
		for (std::size_t k = 1U; k + 1U < poly.size(); ++k)
		{
			const std::size_t i0 = poly[0];
			const std::size_t i1 = poly[k];
			const std::size_t i2 = poly[k + 1U];
			if (i0 >= points.size() || i1 >= points.size() || i2 >= points.size())
			{
				continue;
			}
			pushTriToSoup(soup, points[i0], points[i1], points[i2]);
		}
	}
}

double signedVolumeOfSoupAboutPoint(const std::vector<float>& soup, const double cx, const double cy, const double cz)
{
	double vol = 0.0;
	for (std::size_t i = 0; i + 8U < soup.size(); i += 9U)
	{
		const double ax = static_cast<double>(soup[i]) - cx;
		const double ay = static_cast<double>(soup[i + 1U]) - cy;
		const double az = static_cast<double>(soup[i + 2U]) - cz;
		const double bx = static_cast<double>(soup[i + 3U]) - cx;
		const double by = static_cast<double>(soup[i + 4U]) - cy;
		const double bz = static_cast<double>(soup[i + 5U]) - cz;
		const double ccx = static_cast<double>(soup[i + 6U]) - cx;
		const double ccy = static_cast<double>(soup[i + 7U]) - cy;
		const double ccz = static_cast<double>(soup[i + 8U]) - cz;
		vol += ax * (by * ccz - bz * ccy) - ay * (bx * ccz - bz * ccx) + az * (bx * ccy - by * ccx);
	}
	return vol / 6.0;
}

void flipAllTriangleWindingInSoup(std::vector<float>& soup)
{
	for (std::size_t i = 0; i + 8U < soup.size(); i += 9U)
	{
		std::swap(soup[i + 3U], soup[i + 6U]);
		std::swap(soup[i + 4U], soup[i + 7U]);
		std::swap(soup[i + 5U], soup[i + 8U]);
	}
}

// 封闭体整体内外翻转；逐三角质心翻转在非凸网格上会导致部分面发黑
void orientSoupOutwardIfClosed(std::vector<float>& soup, const double refX, const double refY, const double refZ)
{
	if (soup.size() < 9U)
	{
		return;
	}
	double minX = soup[0];
	double minY = soup[1];
	double minZ = soup[2];
	double maxX = minX;
	double maxY = minY;
	double maxZ = minZ;
	for (std::size_t i = 0; i + 2U < soup.size(); i += 3U)
	{
		minX = std::min(minX, static_cast<double>(soup[i]));
		maxX = std::max(maxX, static_cast<double>(soup[i]));
		minY = std::min(minY, static_cast<double>(soup[i + 1U]));
		maxY = std::max(maxY, static_cast<double>(soup[i + 1U]));
		minZ = std::min(minZ, static_cast<double>(soup[i + 2U]));
		maxZ = std::max(maxZ, static_cast<double>(soup[i + 2U]));
	}
	const double scale = std::max({maxX - minX, maxY - minY, maxZ - minZ, 1e-6});
	const double volEps = scale * scale * scale * 1e-9;
	const double vol = signedVolumeOfSoupAboutPoint(soup, refX, refY, refZ);
	if (vol < -volEps)
	{
		flipAllTriangleWindingInSoup(soup);
	}
}

// Scale-space 输出为展开 soup，须先焊点再 orient_polygon_soup 才能沿共享边传播绕序
bool orientTriangleSoupWinding(std::vector<float>& soup)
{
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		return false;
	}

	namespace PMP = CGAL::Polygon_mesh_processing;

	const auto quantKey = [](const Point_3& p)
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

	const auto vertexIndex = [&](const Point_3& p) -> std::size_t
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
		const Point_3 p0(soup[i], soup[i + 1U], soup[i + 2U]);
		const Point_3 p1(soup[i + 3U], soup[i + 4U], soup[i + 5U]);
		const Point_3 p2(soup[i + 6U], soup[i + 7U], soup[i + 8U]);
		polygons.push_back({vertexIndex(p0), vertexIndex(p1), vertexIndex(p2)});
	}

	if (points.empty() || polygons.empty())
	{
		return false;
	}

	PMP::repair_polygon_soup(points, polygons);
	(void)PMP::orient_polygon_soup(points, polygons);

	const std::vector<float> backup = soup;
	buildSoupFromPolygons(points, polygons, soup);
	if (soup.empty())
	{
		soup = backup;
		return false;
	}

	double refX = 0.0;
	double refY = 0.0;
	double refZ = 0.0;
	for (const Point_3& p : points)
	{
		refX += p.x();
		refY += p.y();
		refZ += p.z();
	}
	const double inv = 1.0 / static_cast<double>(points.size());
	refX *= inv;
	refY *= inv;
	refZ *= inv;
	orientSoupOutwardIfClosed(soup, refX, refY, refZ);
	return true;
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

bool reconstructScaleSpace(const std::vector<float>& xyz, std::vector<float>& triangleSoupOut,
						   const std::size_t smoothIterations, double meshingRadiusMm, std::string* errMsg)
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
	(void)orientTriangleSoupWinding(triangleSoupOut);
	(void)meshingRadiusMm;
	return true;
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
