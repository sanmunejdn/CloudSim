#pragma once

#include <QString>

class MainWindow;

/// 主窗口注册后端对象时的导入、捕获、渲染流水线，协调文件与 OsgWidget 与数据管理器。
class MainWindowImportCaptureRenderController
{
public:
	bool registerBackendObject(MainWindow& mw, const QString& filePath, const QString& typeName,
		bool isPointCloud, bool quietUi);
	/// Loads URDF (meshes + joint origins at zero angle) as a hierarchical mesh assembly.
	bool registerUrdfRobot(MainWindow& mw, const QString& urdfFilePath, bool quietUi);
};

