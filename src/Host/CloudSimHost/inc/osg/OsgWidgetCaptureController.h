#ifndef CLOUDSIMHOST_OSGWIDGETCAPTURECONTROLLER_H
#define CLOUDSIMHOST_OSGWIDGETCAPTURECONTROLLER_H

/// @file OsgWidgetCaptureController.h
/// @brief 从当前 OSG 场景或导入结果中抓取几何，写入后端数据结构（供保存或业务处理）。

#include <QString>
#include <vector>

class OsgWidget;
class PointCloudBackendData;
class MeshBackendData;

struct MeshCapturedPart
{
	QString partPath;
	QString parentPartPath;
	QString displayName;
	std::vector<float> triangleSoup;
};

/// 从当前 OSG 场景或导入结果中抓取几何，写入后端数据结构（供保存或业务处理）。
class OsgWidgetCaptureController
{
public:
	bool captureImportedPointCloudBackend(OsgWidget& self, PointCloudBackendData& out, QString* errorMessage);
	bool captureImportedMeshBackend(OsgWidget& self, MeshBackendData& out, QString* errorMessage);
	bool captureImportedMeshBackendHierarchy(OsgWidget& self, std::vector<MeshCapturedPart>& outParts,
											 QString* errorMessage);
};

#endif // CLOUDSIMHOST_OSGWIDGETCAPTURECONTROLLER_H
