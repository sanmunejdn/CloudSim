/// @file MeshTriangleSelectionUtil.cpp
/// @brief MeshTriangleSelectionUtil 实现

#include "MeshTriangleSelectionUtil.h"

#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"

#include <QVector>

#include <BackendDataManager.h>
#include <MeshBackendData.h>
#include <PointCloudBackendOps.h>
#include <osg/Vec3f>

namespace mesh_triangle_selection
{
bool collectTrianglesByPolyline(IRobotDocumentHost* doc, IRobotOsgViewHost* osg, const std::string& backendIdUtf8,
								const QVector<float>& polylineScreenXy, const QVector<double>& mvpMatrix,
								const int viewportWidth, const int viewportHeight, std::vector<int>& outTriangleIndices,
								std::string* errMsg)
{
	outTriangleIndices.clear();
	if (!doc || !osg)
	{
		if (errMsg)
		{
			*errMsg = "document or osg unavailable";
		}
		return false;
	}
	auto data = doc->backend().getData(backendIdUtf8);
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data);
	if (!mesh || !mesh->hasGeometry())
	{
		if (errMsg)
		{
			*errMsg = "mesh backend unavailable";
		}
		return false;
	}
	double modelToWorld[16] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
	const std::string scopeId = osg->resolvePickScopeBackendId(backendIdUtf8);
	osg::Matrixd worldMat;
	if (osg->getBackendRootWorldMatrix(scopeId, worldMat))
	{
		for (int r = 0; r < 4; ++r)
		{
			for (int c = 0; c < 4; ++c)
			{
				modelToWorld[r + c * 4] = worldMat(r, c);
			}
		}
	}
	std::vector<float> poly;
	poly.reserve(static_cast<std::size_t>(polylineScreenXy.size()));
	for (float v : polylineScreenXy)
	{
		poly.push_back(v);
	}
	if (mvpMatrix.size() < 16)
	{
		if (errMsg)
		{
			*errMsg = "mvp matrix size < 16";
		}
		return false;
	}
	double mvp[16] = {};
	for (int i = 0; i < 16; ++i)
	{
		mvp[i] = mvpMatrix[i];
	}
	return point_cloud_backend_ops::collectMeshTriangleIndicesByPolyline2D(
		*mesh, poly, mvp, modelToWorld, viewportWidth, viewportHeight, true, outTriangleIndices, errMsg);
}

void selectedTrianglesToWorldVerts(const MeshBackendData& mesh, IRobotOsgViewHost* osg,
								   const std::string& backendIdUtf8, const std::vector<int>& triangleIndices,
								   std::vector<osg::Vec3f>& outVertsWorld)
{
	outVertsWorld.clear();
	if (!osg)
	{
		return;
	}
	const auto& soup = mesh.triangleSoup();
	const std::string scopeId = osg->resolvePickScopeBackendId(backendIdUtf8);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(scopeId, worldMat))
	{
		return;
	}
	outVertsWorld.reserve(triangleIndices.size() * 3U);
	for (const int ti : triangleIndices)
	{
		if (ti < 0)
		{
			continue;
		}
		const std::size_t b = static_cast<std::size_t>(ti) * 9U;
		if (b + 8U >= soup.size())
		{
			continue;
		}
		for (int c = 0; c < 3; ++c)
		{
			const std::size_t cb = b + static_cast<std::size_t>(c) * 3U;
			const osg::Vec3d local(soup[cb], soup[cb + 1U], soup[cb + 2U]);
			const osg::Vec3d world = local * worldMat;
			outVertsWorld.emplace_back(static_cast<float>(world.x()), static_cast<float>(world.y()),
									   static_cast<float>(world.z()));
		}
	}
}

void triangleSoupModelToWorldVerts(IRobotOsgViewHost* osg, const std::string& backendIdUtf8,
								   const std::vector<float>& triangleSoupModel, std::vector<osg::Vec3f>& outVertsWorld)
{
	outVertsWorld.clear();
	if (!osg || triangleSoupModel.size() < 9U || triangleSoupModel.size() % 9U != 0U)
	{
		return;
	}
	const std::string scopeId = osg->resolvePickScopeBackendId(backendIdUtf8);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(scopeId, worldMat))
	{
		return;
	}
	const std::size_t triCount = triangleSoupModel.size() / 9U;
	outVertsWorld.reserve(triCount * 3U);
	for (std::size_t ti = 0; ti < triCount; ++ti)
	{
		for (int c = 0; c < 3; ++c)
		{
			const std::size_t cb = ti * 9U + static_cast<std::size_t>(c) * 3U;
			const osg::Vec3d local(triangleSoupModel[cb], triangleSoupModel[cb + 1U], triangleSoupModel[cb + 2U]);
			const osg::Vec3d world = local * worldMat;
			outVertsWorld.emplace_back(static_cast<float>(world.x()), static_cast<float>(world.y()),
									   static_cast<float>(world.z()));
		}
	}
}

} // namespace mesh_triangle_selection
