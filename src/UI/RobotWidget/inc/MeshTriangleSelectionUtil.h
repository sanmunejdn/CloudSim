#ifndef ROBOTWIDGET_MESHTRIANGLESELECTIONUTIL_H
#define ROBOTWIDGET_MESHTRIANGLESELECTIONUTIL_H

/// @file MeshTriangleSelectionUtil.h
/// @brief 屏幕多边形 → 三角面索引（模型系 mesh）

#include "CoreTypes.h"

#include <QVector>
#include <vector>

#include <MeshBackendData.h>
#include <PointCloudBackendOps.h>

class IRobotDocumentHost;
class IRobotOsgViewHost;

namespace mesh_triangle_selection
{
/// 屏幕多边形 → 三角面索引（模型系 mesh）
bool collectTrianglesByPolyline(IRobotDocumentHost* doc, IRobotOsgViewHost* osg, const std::string& backendIdUtf8,
								const QVector<float>& polylineScreenXy, const QVector<double>& mvpMatrix,
								int viewportWidth, int viewportHeight, std::vector<int>& outTriangleIndices,
								std::string* errMsg = nullptr);

/// 选中三角索引 → 世界坐标顶点（供高亮）
void selectedTrianglesToWorldVerts(const MeshBackendData& mesh, IRobotOsgViewHost* osg,
								   const std::string& backendIdUtf8, const std::vector<int>& triangleIndices,
								   std::vector<cloudsim::core::Vec3>& outVertsWorld);

/// 模型坐标三角 soup → 世界坐标顶点
void triangleSoupModelToWorldVerts(IRobotOsgViewHost* osg, const std::string& backendIdUtf8,
								   const std::vector<float>& triangleSoupModel,
								   std::vector<cloudsim::core::Vec3>& outVertsWorld);

} // namespace mesh_triangle_selection

#endif // ROBOTWIDGET_MESHTRIANGLESELECTIONUTIL_H
