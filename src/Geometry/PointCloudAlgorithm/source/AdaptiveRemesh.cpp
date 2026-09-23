/// @file AdaptiveRemesh.cpp
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com

#include "AdaptiveRemesh.h"

#include "KdTreePointSet.h"

#include <MeshRemesh.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/tangential_relaxation.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>

namespace pclalgo
{
namespace
{
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using V = Mesh::Vertex_index;
using E = Mesh::Edge_index;
using H = Mesh::Halfedge_index;
using F = Mesh::Face_index;
namespace PMP = CGAL::Polygon_mesh_processing;

constexpr double kEps = 1e-12;
constexpr double kSplitRatio = 4.0 / 3.0;
constexpr double kCollapseRatio = 4.0 / 5.0;

struct ResidualField
{
	KdTreePointSet tree;
	std::vector<float> values;
	double scaleC = 0.0;
	double floorMm = 1e-6;
	bool active = false;

	double lookup(const double x, const double y, const double z) const
	{
		if (!active)
		{
			return 0.0;
		}
		double distSq = 0.0;
		const std::size_t nn =
			tree.findNearest(x, y, z, std::numeric_limits<double>::max(), distSq);
		if (nn == static_cast<std::size_t>(-1) || nn >= values.size())
		{
			return 0.0;
		}
		return static_cast<double>(values[nn]);
	}
};

bool resolveBounds(const AdaptiveRemeshParams& params, double& eps, double& lMin, double& lMax, std::string* errMsg)
{
	const double h = params.characteristicEdgeMm;
	eps = params.approxTolMm > 0.0 ? params.approxTolMm : (h > 0.0 ? 0.02 * h : 0.0);
	lMin = params.edgeMinMm > 0.0 ? params.edgeMinMm : (h > 0.0 ? 0.25 * h : 0.0);
	lMax = params.edgeMaxMm > 0.0 ? params.edgeMaxMm : (h > 0.0 ? 2.0 * h : 0.0);
	if (!(lMin > 0.0) || !(lMax > 0.0) || !(eps > 0.0) || lMin > lMax)
	{
		if (errMsg)
		{
			*errMsg = "adaptive remesh: invalid edge/tolerance bounds";
		}
		return false;
	}
	return true;
}

void buildResidualField(const AdaptiveRemeshParams& params, const double characteristicH, ResidualField& out)
{
	out = ResidualField{};
	out.floorMm = params.residualFloorMm > 0.0 ? params.residualFloorMm : 1e-6;
	const std::size_t n = params.residualMm.size();
	if (n == 0U || params.residualSampleXyz.size() != n * 3U)
	{
		return;
	}
	out.tree.build(params.residualSampleXyz);
	out.values = params.residualMm;
	if (out.tree.empty())
	{
		return;
	}

	std::vector<double> sorted;
	sorted.reserve(n);
	for (const float e : params.residualMm)
	{
		if (std::isfinite(e) && e >= 0.0f)
		{
			sorted.push_back(static_cast<double>(e));
		}
	}
	if (sorted.empty())
	{
		return;
	}
	std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2U), sorted.end());
	const double medianE = sorted[sorted.size() / 2U];

	if (params.residualEdgeScale > 0.0)
	{
		out.scaleC = params.residualEdgeScale;
	}
	else
	{
		const double h = characteristicH > 0.0 ? characteristicH : 1.0;
		out.scaleC = h * std::sqrt(medianE + out.floorMm);
	}
	out.active = out.scaleC > 0.0;
}

bool soupToMesh(const std::vector<float>& soup, Mesh& mesh, std::string* errMsg)
{
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "adaptive remesh: invalid triangle soup";
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
		const Point_3 p0(soup[i], soup[i + 1U], soup[i + 2U]);
		const Point_3 p1(soup[i + 3U], soup[i + 4U], soup[i + 5U]);
		const Point_3 p2(soup[i + 6U], soup[i + 7U], soup[i + 8U]);
		polygons.push_back({vertexIndex(p0), vertexIndex(p1), vertexIndex(p2)});
	}

	PMP::repair_polygon_soup(points, polygons);
	(void)PMP::orient_polygon_soup(points, polygons);
	mesh.clear();
	PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
	if (mesh.is_empty() || !CGAL::is_triangle_mesh(mesh))
	{
		if (errMsg)
		{
			*errMsg = "adaptive remesh: soup to mesh failed";
		}
		return false;
	}
	return true;
}

void meshToSoup(const Mesh& mesh, std::vector<float>& soupOut)
{
	soupOut.clear();
	soupOut.reserve(static_cast<std::size_t>(mesh.number_of_faces()) * 9U);
	for (const F f : mesh.faces())
	{
		if (mesh.is_removed(f))
		{
			continue;
		}
		H h = mesh.halfedge(f);
		if (CGAL::halfedges_around_face(h, mesh).size() != 3)
		{
			continue;
		}
		for (int i = 0; i < 3; ++i)
		{
			const Point_3& p = mesh.point(mesh.target(h));
			soupOut.push_back(static_cast<float>(p.x()));
			soupOut.push_back(static_cast<float>(p.y()));
			soupOut.push_back(static_cast<float>(p.z()));
			h = mesh.next(h);
		}
	}
}

double edgeLength(const Mesh& mesh, const E e)
{
	const H h = mesh.halfedge(e);
	return std::sqrt(CGAL::squared_distance(mesh.point(mesh.source(h)), mesh.point(mesh.target(h))));
}

double dihedralAngleDeg(const Mesh& mesh, const E e)
{
	const H h = mesh.halfedge(e);
	if (mesh.is_border(h) || mesh.is_border(mesh.opposite(h)))
	{
		return 0.0;
	}
	const F f0 = mesh.face(h);
	const F f1 = mesh.face(mesh.opposite(h));
	if (f0 == Mesh::null_face() || f1 == Mesh::null_face() || mesh.is_removed(f0) || mesh.is_removed(f1))
	{
		return 0.0;
	}
	// 非三角面（例如只 split_edge 未 split_face）上 compute_face_normal 会踩坏数据
	if (CGAL::halfedges_around_face(h, mesh).size() != 3 ||
		CGAL::halfedges_around_face(mesh.opposite(h), mesh).size() != 3)
	{
		return 0.0;
	}
	const Vector_3 n0 = PMP::compute_face_normal(f0, mesh);
	const Vector_3 n1 = PMP::compute_face_normal(f1, mesh);
	const double l0 = n0.squared_length();
	const double l1 = n1.squared_length();
	if (l0 < kEps || l1 < kEps)
	{
		return 0.0;
	}
	double c = (n0 * n1) / std::sqrt(l0 * l1);
	c = std::max(-1.0, std::min(1.0, c));
	return std::acos(c) * (180.0 / 3.14159265358979323846);
}

// 一环法向差分 / 边长 → 近似 |κ_max|
void estimateMaxCurvature(Mesh& mesh, std::unordered_map<V, double>& kappaOut)
{
	kappaOut.clear();
	auto vnormals = mesh.add_property_map<V, Vector_3>("v:adaptive_normal", CGAL::NULL_VECTOR).first;
	PMP::compute_vertex_normals(mesh, vnormals);

	for (const V v : mesh.vertices())
	{
		double kMax = 0.0;
		const Vector_3 nv = vnormals[v];
		const Point_3 pv = mesh.point(v);
		for (const V u : CGAL::vertices_around_target(mesh.halfedge(v), mesh))
		{
			if (u == v)
			{
				continue;
			}
			const double len = std::sqrt(CGAL::squared_distance(pv, mesh.point(u)));
			if (len < kEps)
			{
				continue;
			}
			const Vector_3 nu = vnormals[u];
			const double dn = std::sqrt((nv - nu).squared_length());
			kMax = std::max(kMax, dn / len);
		}
		kappaOut[v] = kMax;
	}
	mesh.remove_property_map(vnormals);
}

double targetLengthAt(const Mesh& mesh, const V v, const std::unordered_map<V, double>& kappa, const double eps,
					  const double lMin, const double lMax, const ResidualField* residual)
{
	const auto it = kappa.find(v);
	const double k = (it != kappa.end()) ? it->second : 0.0;
	const double rawK = std::sqrt(6.0 * eps / (k + 1e-9));
	double raw = rawK;
	if (residual && residual->active)
	{
		const Point_3& p = mesh.point(v);
		const double e = residual->lookup(p.x(), p.y(), p.z());
		const double rawE = residual->scaleC / std::sqrt(e + residual->floorMm);
		raw = std::min(rawK, rawE);
	}
	return std::max(lMin, std::min(lMax, raw));
}

double targetLengthEdge(const Mesh& mesh, const E e, const std::unordered_map<V, double>& kappa, const double eps,
						const double lMin, const double lMax, const ResidualField* residual)
{
	const H h = mesh.halfedge(e);
	const double la = targetLengthAt(mesh, mesh.source(h), kappa, eps, lMin, lMax, residual);
	const double lb = targetLengthAt(mesh, mesh.target(h), kappa, eps, lMin, lMax, residual);
	return 0.5 * (la + lb);
}

bool isFeatureEdge(const Mesh& mesh, const E e, const double featureAngleDeg)
{
	return dihedralAngleDeg(mesh, e) >= featureAngleDeg;
}

void splitLongEdges(Mesh& mesh, const std::unordered_map<V, double>& kappa, const double eps, const double lMin,
					const double lMax, const ResidualField* residual)
{
	std::vector<E> longEdges;
	longEdges.reserve(mesh.number_of_edges());
	for (const E e : mesh.edges())
	{
		const double len = edgeLength(mesh, e);
		const double lt = targetLengthEdge(mesh, e, kappa, eps, lMin, lMax, residual);
		if (len > kSplitRatio * lt)
		{
			longEdges.push_back(e);
		}
	}
	// 长边优先，避免一次迭代里邻边互相干扰过多
	std::sort(longEdges.begin(), longEdges.end(),
			  [&](const E a, const E b) { return edgeLength(mesh, a) > edgeLength(mesh, b); });

	for (const E e : longEdges)
	{
		if (!mesh.has_valid_index(e) || mesh.is_removed(e))
		{
			continue;
		}
		const double len = edgeLength(mesh, e);
		const double lt = targetLengthEdge(mesh, e, kappa, eps, lMin, lMax, residual);
		if (len <= kSplitRatio * lt)
		{
			continue;
		}
		const H h = mesh.halfedge(e);
		const Point_3 mid = CGAL::midpoint(mesh.point(mesh.source(h)), mesh.point(mesh.target(h)));
		const H hNew = CGAL::Euler::split_edge(h, mesh);
		mesh.point(mesh.target(hNew)) = mid;
		// split_edge 会使邻面变四边形，须再 split_face 才能保持三角网格
		if (!mesh.is_border(hNew))
		{
			CGAL::Euler::split_face(hNew, mesh.next(mesh.next(hNew)), mesh);
		}
		const H hOpp = mesh.opposite(hNew);
		if (!mesh.is_border(hOpp))
		{
			CGAL::Euler::split_face(mesh.prev(hOpp), mesh.next(hOpp), mesh);
		}
	}
}

void collapseShortEdges(Mesh& mesh, const std::unordered_map<V, double>& kappa, const double eps, const double lMin,
						const double lMax, const double featureAngleDeg, const ResidualField* residual)
{
	std::vector<E> shortEdges;
	shortEdges.reserve(mesh.number_of_edges());
	for (const E e : mesh.edges())
	{
		if (isFeatureEdge(mesh, e, featureAngleDeg))
		{
			continue;
		}
		const double len = edgeLength(mesh, e);
		const double lt = targetLengthEdge(mesh, e, kappa, eps, lMin, lMax, residual);
		if (len < kCollapseRatio * lt)
		{
			shortEdges.push_back(e);
		}
	}
	std::sort(shortEdges.begin(), shortEdges.end(),
			  [&](const E a, const E b) { return edgeLength(mesh, a) < edgeLength(mesh, b); });

	for (const E e : shortEdges)
	{
		if (!mesh.has_valid_index(e) || mesh.is_removed(e))
		{
			continue;
		}
		if (isFeatureEdge(mesh, e, featureAngleDeg))
		{
			continue;
		}
		const double len = edgeLength(mesh, e);
		const double lt = targetLengthEdge(mesh, e, kappa, eps, lMin, lMax, residual);
		if (len >= kCollapseRatio * lt)
		{
			continue;
		}
		if (!CGAL::Euler::does_satisfy_link_condition(e, mesh))
		{
			continue;
		}
		CGAL::Euler::collapse_edge(e, mesh);
	}
	mesh.collect_garbage();
}

void equalizeValence(Mesh& mesh, const double featureAngleDeg)
{
	std::vector<E> edges(mesh.edges().begin(), mesh.edges().end());
	for (const E e : edges)
	{
		if (!mesh.has_valid_index(e) || mesh.is_removed(e) || mesh.is_border(e))
		{
			continue;
		}
		if (isFeatureEdge(mesh, e, featureAngleDeg))
		{
			continue;
		}
		const H h = mesh.halfedge(e);
		const V v0 = mesh.source(h);
		const V v1 = mesh.target(h);
		const V v2 = mesh.target(mesh.next(h));
		const V v3 = mesh.target(mesh.next(mesh.opposite(h)));
		const int d0 = static_cast<int>(mesh.degree(v0));
		const int d1 = static_cast<int>(mesh.degree(v1));
		const int d2 = static_cast<int>(mesh.degree(v2));
		const int d3 = static_cast<int>(mesh.degree(v3));
		const int before = std::abs(d0 - 6) + std::abs(d1 - 6) + std::abs(d2 - 6) + std::abs(d3 - 6);
		const int after = std::abs(d0 - 6 - 1) + std::abs(d1 - 6 - 1) + std::abs(d2 - 6 + 1) + std::abs(d3 - 6 + 1);
		if (after < before)
		{
			CGAL::Euler::flip_edge(h, mesh);
		}
	}
}

} // namespace

bool adaptiveIsotropicRemesh(const std::vector<float>& triangleSoupIn, std::vector<float>& triangleSoupOut,
							 const AdaptiveRemeshParams& params, std::string* errMsg)
{
	triangleSoupOut.clear();
	double eps = 0.0;
	double lMin = 0.0;
	double lMax = 0.0;
	if (!resolveBounds(params, eps, lMin, lMax, errMsg))
	{
		return false;
	}

	ResidualField residual;
	buildResidualField(params, params.characteristicEdgeMm > 0.0 ? params.characteristicEdgeMm : lMax, residual);

	std::vector<float> baseSoup;
	if (!vcgalgo::isotropicRemesh(triangleSoupIn, lMax, baseSoup, params.baseRemeshIterations, params.featureAngleDeg,
								  errMsg))
	{
		if (errMsg && errMsg->empty())
		{
			*errMsg = "adaptive remesh: base isotropic remesh failed";
		}
		return false;
	}

	Mesh mesh;
	if (!soupToMesh(baseSoup, mesh, errMsg))
	{
		return false;
	}

	const ResidualField* residualPtr = residual.active ? &residual : nullptr;
	const int iters = std::max(1, params.refineIterations);
	for (int i = 0; i < iters; ++i)
	{
		std::unordered_map<V, double> kappa;
		estimateMaxCurvature(mesh, kappa);
		splitLongEdges(mesh, kappa, eps, lMin, lMax, residualPtr);
		estimateMaxCurvature(mesh, kappa);
		collapseShortEdges(mesh, kappa, eps, lMin, lMax, params.featureAngleDeg, residualPtr);
		equalizeValence(mesh, params.featureAngleDeg);
		PMP::tangential_relaxation(mesh);
	}

	if (mesh.is_empty() || mesh.number_of_faces() == 0)
	{
		if (errMsg)
		{
			*errMsg = "adaptive remesh: empty output";
		}
		return false;
	}

	meshToSoup(mesh, triangleSoupOut);
	return !triangleSoupOut.empty();
}

} // namespace pclalgo
