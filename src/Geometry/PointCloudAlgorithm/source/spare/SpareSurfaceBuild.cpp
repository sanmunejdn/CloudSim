/// @file SpareSurfaceBuild.cpp
/// @brief SpareSurfaceBuild 实现

#include "KdTreePointSet.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"
#include "spare/SpareSurface.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

namespace pclalgo
{
namespace spare
{
namespace
{
using CgalKernel = CGAL::Simple_cartesian<double>;
using CgalPoint = CgalKernel::Point_3;
using CgalMesh = CGAL::Surface_mesh<CgalPoint>;
namespace PMP = CGAL::Polygon_mesh_processing;

Vector3 toVec3(const CgalPoint& p)
{
	return Vector3(p.x(), p.y(), p.z());
}

bool soupToCgalMesh(const std::vector<float>& soup, CgalMesh& mesh, std::string* errMsg)
{
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid triangle soup";
		}
		return false;
	}

	auto quantKey = [](const CgalPoint& p)
	{
		return std::make_tuple(static_cast<long long>(std::llround(p.x() / 1e-4)),
							   static_cast<long long>(std::llround(p.y() / 1e-4)),
							   static_cast<long long>(std::llround(p.z() / 1e-4)));
	};

	std::map<std::tuple<long long, long long, long long>, std::size_t> pointIndex;
	std::vector<CgalPoint> points;
	std::vector<std::vector<std::size_t>> polygons;
	points.reserve(soup.size() / 3U);
	polygons.reserve(soup.size() / 9U);

	auto vertexIndex = [&](const CgalPoint& p) -> std::size_t
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
		const CgalPoint p0(soup[i], soup[i + 1U], soup[i + 2U]);
		const CgalPoint p1(soup[i + 3U], soup[i + 4U], soup[i + 5U]);
		const CgalPoint p2(soup[i + 6U], soup[i + 7U], soup[i + 8U]);
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
			*errMsg = "polygon_soup_to_polygon_mesh failed";
		}
		return false;
	}
	return true;
}

void buildMeshTopology(SpareSurface& out, const CgalMesh& mesh)
{
	const std::size_t n = mesh.number_of_vertices();
	out.positions.resize(n);
	out.normals.resize(n, Vector3::Zero());
	out.vertexNeighbors.assign(n, {});
	out.halfEdges.clear();
	out.faces.clear();

	std::unordered_map<CgalMesh::Vertex_index, int> vmap;
	int vid = 0;
	for (const auto v : mesh.vertices())
	{
		vmap[v] = vid;
		out.positions[static_cast<std::size_t>(vid)] = toVec3(mesh.point(v));
		++vid;
	}

	for (const auto f : mesh.faces())
	{
		std::array<int, 3> tri{};
		int c = 0;
		for (const auto v : vertices_around_face(mesh.halfedge(f), mesh))
		{
			if (c < 3)
			{
				tri[static_cast<std::size_t>(c)] = vmap[v];
			}
			++c;
		}
		if (c == 3)
		{
			out.faces.push_back(tri);
		}
	}

	for (const auto h : mesh.halfedges())
	{
		const auto v0 = mesh.source(h);
		const auto v1 = mesh.target(h);
		const int i0 = vmap[v0];
		const int i1 = vmap[v1];
		out.halfEdges.emplace_back(i0, i1);
		out.vertexNeighbors[static_cast<std::size_t>(i0)].push_back(i1);
	}

	for (std::size_t i = 0; i < n; ++i)
	{
		auto& nbrs = out.vertexNeighbors[i];
		std::sort(nbrs.begin(), nbrs.end());
		nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
	}

	for (const auto v : mesh.vertices())
	{
		const int idx = vmap[v];
		const auto nrm = PMP::compute_vertex_normal(v, mesh);
		Vector3 normal(nrm.x(), nrm.y(), nrm.z());
		const double len = normal.norm();
		if (len > 1e-12)
		{
			normal /= len;
		}
		out.normals[static_cast<std::size_t>(idx)] = normal;
	}
}

void populateKnnNeighbors(SpareSurface& surface, const unsigned int k)
{
	const std::size_t n = surface.vertexCount();
	if (n == 0U)
	{
		return;
	}

	std::vector<float> xyz;
	xyz.reserve(n * 3U);
	for (std::size_t i = 0; i < n; ++i)
	{
		xyz.push_back(static_cast<float>(surface.positions[i].x()));
		xyz.push_back(static_cast<float>(surface.positions[i].y()));
		xyz.push_back(static_cast<float>(surface.positions[i].z()));
	}

	KdTreePointSet tree(xyz);
	surface.vertexNeighbors.assign(n, {});
	std::vector<std::size_t> indices;
	std::vector<double> distSq;
	for (std::size_t i = 0; i < n; ++i)
	{
		tree.findKNearest(surface.positions[i].x(), surface.positions[i].y(), surface.positions[i].z(), k + 1U, indices,
						  distSq);
		for (std::size_t j = 0; j < indices.size(); ++j)
		{
			if (indices[j] != i)
			{
				surface.vertexNeighbors[i].push_back(static_cast<int>(indices[j]));
			}
		}
	}
}

} // namespace

bool buildSpareSurfaceFromXyz(SpareSurface& out, const std::vector<float>& xyz, const std::vector<float>* normals,
							  const bool buildKnnGraph, std::string* errMsg)
{
	if (!validXyzLength(xyz))
	{
		if (errMsg)
		{
			*errMsg = "invalid xyz buffer";
		}
		return false;
	}

	const std::size_t n = pointCountFromXyz(xyz);
	out.positions.resize(n);
	out.normals.resize(n);
	out.faces.clear();
	out.halfEdges.clear();

	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		out.positions[i] = Vector3(xyz[b], xyz[b + 1U], xyz[b + 2U]);
	}

	if (normals != nullptr && normals->size() == n * 3U)
	{
		for (std::size_t i = 0; i < n; ++i)
		{
			const std::size_t b = i * 3U;
			out.normals[i] = Vector3((*normals)[b], (*normals)[b + 1U], (*normals)[b + 2U]);
			const double len = out.normals[i].norm();
			if (len > 1e-12)
			{
				out.normals[i] /= len;
			}
		}
	}
	else
	{
		out.normals.assign(n, Vector3(0.0, 0.0, 1.0));
	}

	if (buildKnnGraph)
	{
		populateKnnNeighbors(out, 6U);
	}
	else
	{
		out.vertexNeighbors.assign(n, {});
	}
	return true;
}

bool buildSpareSurfaceFromMeshSoup(SpareSurface& out, const std::vector<float>& triangleSoup, std::string* errMsg)
{
	CgalMesh mesh;
	if (!soupToCgalMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}
	buildMeshTopology(out, mesh);

	out.soupCornerToVertex.assign(triangleSoup.size() / 3U, -1);
	auto quantKey = [](const float x, const float y, const float z)
	{
		return std::make_tuple(static_cast<long long>(std::llround(x / 1e-4f)),
							   static_cast<long long>(std::llround(y / 1e-4f)),
							   static_cast<long long>(std::llround(z / 1e-4f)));
	};
	std::map<std::tuple<long long, long long, long long>, int> weldedToSurface;
	for (std::size_t i = 0; i < out.vertexCount(); ++i)
	{
		const auto key = quantKey(static_cast<float>(out.positions[i].x()), static_cast<float>(out.positions[i].y()),
								  static_cast<float>(out.positions[i].z()));
		weldedToSurface[key] = static_cast<int>(i);
	}
	for (std::size_t corner = 0; corner < out.soupCornerToVertex.size(); ++corner)
	{
		const std::size_t b = corner * 3U;
		const auto key = quantKey(triangleSoup[b], triangleSoup[b + 1U], triangleSoup[b + 2U]);
		const auto it = weldedToSurface.find(key);
		if (it != weldedToSurface.end())
		{
			out.soupCornerToVertex[corner] = it->second;
		}
	}
	return true;
}

bool ensureSpareSurfaceNormals(SpareSurface& surface, std::string* errMsg)
{
	if (surface.vertexCount() == 0U)
	{
		if (errMsg)
		{
			*errMsg = "empty surface";
		}
		return false;
	}

	bool needEstimate = false;
	for (const Vector3& n : surface.normals)
	{
		if (n.norm() < 1e-9)
		{
			needEstimate = true;
			break;
		}
	}
	if (!needEstimate)
	{
		return true;
	}

	std::vector<float> xyz;
	std::vector<float> normals;
	spareSurfaceToXyz(surface, xyz, normals);
	if (!estimateNormalsPca(xyz, normals, 12U, errMsg))
	{
		return false;
	}
	if (!orientNormalsMst(xyz, normals, 12U, nullptr, errMsg))
	{
		return false;
	}

	for (std::size_t i = 0; i < surface.vertexCount(); ++i)
	{
		const std::size_t b = i * 3U;
		surface.normals[i] = Vector3(normals[b], normals[b + 1U], normals[b + 2U]);
	}
	return true;
}

void spareSurfaceToXyz(const SpareSurface& surface, std::vector<float>& xyzOut, std::vector<float>& normalsOut)
{
	const std::size_t n = surface.vertexCount();
	xyzOut.resize(n * 3U);
	normalsOut.resize(n * 3U);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		xyzOut[b] = static_cast<float>(surface.positions[i].x());
		xyzOut[b + 1U] = static_cast<float>(surface.positions[i].y());
		xyzOut[b + 2U] = static_cast<float>(surface.positions[i].z());
		normalsOut[b] = static_cast<float>(surface.normals[i].x());
		normalsOut[b + 1U] = static_cast<float>(surface.normals[i].y());
		normalsOut[b + 2U] = static_cast<float>(surface.normals[i].z());
	}
}

void spareSurfaceToMeshSoup(const SpareSurface& surface, const std::vector<float>& originalSoup,
							std::vector<float>& soupOut)
{
	soupOut = originalSoup;
	if (surface.soupCornerToVertex.size() != originalSoup.size() / 3U)
	{
		return;
	}
	for (std::size_t corner = 0; corner < surface.soupCornerToVertex.size(); ++corner)
	{
		const int vidx = surface.soupCornerToVertex[corner];
		if (vidx < 0)
		{
			continue;
		}
		const std::size_t b = corner * 3U;
		const Vector3& p = surface.positions[static_cast<std::size_t>(vidx)];
		soupOut[b] = static_cast<float>(p.x());
		soupOut[b + 1U] = static_cast<float>(p.y());
		soupOut[b + 2U] = static_cast<float>(p.z());
	}
}

Scalar normalizeSpareSurfaces(SpareSurface& source, SpareSurface& target)
{
	Eigen::AlignedBox3d box;
	for (const Vector3& p : source.positions)
	{
		box.extend(p);
	}
	for (const Vector3& p : target.positions)
	{
		box.extend(p);
	}
	const double diag = box.diagonal().norm();
	if (diag < 1e-12)
	{
		return 1.0;
	}
	const Scalar scale = static_cast<Scalar>(1.0 / diag);
	applyScaleToSpareSurface(source, scale);
	applyScaleToSpareSurface(target, scale);
	return scale;
}

void applyScaleToSpareSurface(SpareSurface& surface, const Scalar scale)
{
	for (Vector3& p : surface.positions)
	{
		p *= scale;
	}
}

} // namespace spare
} // namespace pclalgo
