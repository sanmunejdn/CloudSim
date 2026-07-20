/// @file MeshSurfaceReconstructionGmcgUvPack.cpp
/// @brief MeshSurfaceReconstructionGmcgUvPack 实现

#include "MeshSurfaceReconstructionGmcgUvPack.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace geoalgo
{
namespace meshrecon
{
namespace
{
struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

Vec3 readV(const std::vector<float>& verts, const int vi)
{
	const std::size_t b = static_cast<std::size_t>(vi) * 3U;
	return {verts[b], verts[b + 1U], verts[b + 2U]};
}

} // namespace

void packChartUvCoordinates(const QuadMeshLite& globalQuad, const GmcgQuadGraph& graph,
							const GmcgChartFaceGroups& groups, std::vector<QuadMeshLite>& chartMeshes,
							std::vector<std::vector<float>>& chartVertexUv)
{
	(void)graph;
	chartMeshes.clear();
	chartVertexUv.clear();
	chartMeshes.reserve(groups.chartFaces.size());
	chartVertexUv.reserve(groups.chartFaces.size());

	for (const std::vector<int>& faces : groups.chartFaces)
	{
		if (faces.empty())
		{
			continue;
		}
		QuadMeshLite chartMesh;
		chartMesh.vertices = globalQuad.vertices;
		std::unordered_set<int> usedVerts;
		for (const int fi : faces)
		{
			const std::size_t b = static_cast<std::size_t>(fi) * 4U;
			for (int k = 0; k < 4; ++k)
			{
				const int vi = globalQuad.quadFaces[b + static_cast<std::size_t>(k)];
				chartMesh.quadFaces.push_back(vi);
				usedVerts.insert(vi);
			}
		}

		Vec3 mean{};
		for (const int vi : usedVerts)
		{
			const Vec3 p = readV(chartMesh.vertices, vi);
			mean.x += p.x;
			mean.y += p.y;
			mean.z += p.z;
		}
		const double invN = 1.0 / std::max<std::size_t>(1U, usedVerts.size());
		mean.x *= invN;
		mean.y *= invN;
		mean.z *= invN;

		double xx = 0.0;
		double xy = 0.0;
		double xz = 0.0;
		double yy = 0.0;
		double yz = 0.0;
		double zz = 0.0;
		for (const int vi : usedVerts)
		{
			const Vec3 p = readV(chartMesh.vertices, vi);
			const double dx = p.x - mean.x;
			const double dy = p.y - mean.y;
			const double dz = p.z - mean.z;
			xx += dx * dx;
			xy += dx * dy;
			xz += dx * dz;
			yy += dy * dy;
			yz += dy * dz;
			zz += dz * dz;
		}
		// 第一主轴作 u 方向，次轴作 v
		Vec3 axisU{xx, xy, xz};
		Vec3 axisV{xy, yy, yz};
		const double lenU = std::sqrt(axisU.x * axisU.x + axisU.y * axisU.y + axisU.z * axisU.z);
		const double lenV = std::sqrt(axisV.x * axisV.x + axisV.y * axisV.y + axisV.z * axisV.z);
		if (lenU > 1e-12)
		{
			axisU.x /= lenU;
			axisU.y /= lenU;
			axisU.z /= lenU;
		}
		else
		{
			axisU = {1.0, 0.0, 0.0};
		}
		if (lenV > 1e-12)
		{
			axisV.x /= lenV;
			axisV.y /= lenV;
			axisV.z /= lenV;
		}
		else
		{
			axisV = {0.0, 1.0, 0.0};
		}

		double minU = std::numeric_limits<double>::max();
		double minV = std::numeric_limits<double>::max();
		double maxU = std::numeric_limits<double>::lowest();
		double maxV = std::numeric_limits<double>::lowest();
		std::vector<float> uv(chartMesh.vertices.size() / 3U * 2U, 0.f);
		for (const int vi : usedVerts)
		{
			const Vec3 p = readV(chartMesh.vertices, vi);
			const double dx = p.x - mean.x;
			const double dy = p.y - mean.y;
			const double dz = p.z - mean.z;
			const double u = dx * axisU.x + dy * axisU.y + dz * axisU.z;
			const double v = dx * axisV.x + dy * axisV.y + dz * axisV.z;
			minU = std::min(minU, u);
			minV = std::min(minV, v);
			maxU = std::max(maxU, u);
			maxV = std::max(maxV, v);
			uv[static_cast<std::size_t>(vi) * 2U] = static_cast<float>(u);
			uv[static_cast<std::size_t>(vi) * 2U + 1U] = static_cast<float>(v);
		}
		const double du = std::max(1e-9, maxU - minU);
		const double dv = std::max(1e-9, maxV - minV);
		for (const int vi : usedVerts)
		{
			uv[static_cast<std::size_t>(vi) * 2U] =
				static_cast<float>((uv[static_cast<std::size_t>(vi) * 2U] - static_cast<float>(minU)) / du);
			uv[static_cast<std::size_t>(vi) * 2U + 1U] =
				static_cast<float>((uv[static_cast<std::size_t>(vi) * 2U + 1U] - static_cast<float>(minV)) / dv);
		}
		chartMesh.vertexUv = uv;
		chartMeshes.push_back(std::move(chartMesh));
		chartVertexUv.push_back(std::move(uv));
	}
}

} // namespace meshrecon
} // namespace geoalgo
