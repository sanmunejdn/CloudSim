#ifndef ROBOTURDF_CUSTOMDEVICEURDFEXPORTER_H
#define ROBOTURDF_CUSTOMDEVICEURDFEXPORTER_H

/// @file CustomDeviceUrdfExporter.h
/// @brief 自定义设备 → ROS 包（package.xml + urdf + meshes/cad）

#include "robot_urdf_global.h"

#include <QMap>
#include <QString>

class BackendDataManager;
class CustomDeviceBackendData;

struct ROBOT_URDF_API CustomDeviceUrdfExportOptions
{
	/// 用户所选父目录；其下创建 <packageName>/
	QString packageParentDir;
	/// 空则从设备名清洗
	QString packageName;
	/// geometryBackendId → 源文件绝对路径（DocumentHost sidecar）
	QMap<QString, QString> sourcePathByBackendId;
};

struct ROBOT_URDF_API CustomDeviceUrdfExportResult
{
	bool ok = false;
	QString packageRoot;
	QString urdfPath;
	QString error;
};

/// 写出可被现有 importUrdfRobot 回灌的包；.urdf 长度单位为米
ROBOT_URDF_API CustomDeviceUrdfExportResult
exportCustomDeviceUrdfPackage(const CustomDeviceBackendData& device, const BackendDataManager& backend,
							  const CustomDeviceUrdfExportOptions& options);

#endif // ROBOTURDF_CUSTOMDEVICEURDFEXPORTER_H
