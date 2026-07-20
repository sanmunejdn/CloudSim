#ifndef CLOUDSIMHOST_OSGWIDGETBACKENDLOADCONTROLLER_H
#define CLOUDSIMHOST_OSGWIDGETBACKENDLOADCONTROLLER_H

/// @file OsgWidgetBackendLoadController.h
/// @brief 将后端数据（点云或网格）构建为 OSG 场景节点并挂接到当前视图，负责数据到渲染的转换。

#include <QString>

class OsgWidget;
class PointCloudBackendData;
class MeshBackendData;

/// 将后端数据（点云或网格）构建为 OSG 场景节点并挂接到当前视图，负责数据到渲染的转换。
class OsgWidgetBackendLoadController
{
public:
	bool loadPointCloudFromBackendData(OsgWidget& self, const PointCloudBackendData& data, QString* errorMessage,
									   bool resetViewToHome);
	bool loadMeshFromBackendData(OsgWidget& self, const MeshBackendData& data, QString* errorMessage,
								 bool resetViewToHome, bool showWireOutline = true, bool useSceneLighting = true);
};

#endif // CLOUDSIMHOST_OSGWIDGETBACKENDLOADCONTROLLER_H
