#ifndef WIDGET_MAINWINDOWIMPORTCAPTURERENDERCONTROLLER_H
#define WIDGET_MAINWINDOWIMPORTCAPTURERENDERCONTROLLER_H

/// @file MainWindowImportCaptureRenderController.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 菜单/工程导入编排：Host Facade + 大文件 Job + URDF

#include <QString>

class MainWindow;

/// 菜单/工程导入编排：Host Facade + 大文件 Job + URDF
class MainWindowImportCaptureRenderController
{
public:
	bool registerBackendObject(MainWindow& mw, const QString& filePath, const QString& typeName, bool isPointCloud,
							   bool quietUi);
	/// URDF 零位姿层级导入
	bool registerUrdfRobot(MainWindow& mw, const QString& urdfFilePath, bool quietUi);
};

#endif // WIDGET_MAINWINDOWIMPORTCAPTURERENDERCONTROLLER_H
