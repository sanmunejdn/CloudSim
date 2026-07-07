#pragma once

#include <MeshBackendData.h>
#include <PointCloudBackendOps.h>

#include <QVector>

#include <vector>

#include <osg/Vec3f>

class IRobotDocumentHost;
class IRobotOsgViewHost;

namespace mesh_triangle_selection
{

/// 屏幕多边形 → 三角面索引（模型系 mesh）
bool collectTrianglesByPolyline(
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	const std::string& backendIdUtf8,
	const QVector<float>& polylineScreenXy,
	const QVector<double>& mvpMatrix,
	int viewportWidth,
	int viewportHeight,
	std::vector<int>& outTriangleIndices,
	std::string* errMsg = nullptr);

/// 选中三角索引 → 世界坐标顶点（供高亮）
void selectedTrianglesToWorldVerts(
	const MeshBackendData& mesh,
	IRobotOsgViewHost* osg,
	const std::string& backendIdUtf8,
	const std::vector<int>& triangleIndices,
	std::vector<osg::Vec3f>& outVertsWorld);

/// 模型坐标三角 soup → 世界坐标顶点
void triangleSoupModelToWorldVerts(
	IRobotOsgViewHost* osg,
	const std::string& backendIdUtf8,
	const std::vector<float>& triangleSoupModel,
	std::vector<osg::Vec3f>& outVertsWorld);

} // namespace mesh_triangle_selection
