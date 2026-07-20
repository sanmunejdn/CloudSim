#ifndef CLOUDSIMHOST_OSGWIDGETCAMERAFOCUSCONTROLLER_H
#define CLOUDSIMHOST_OSGWIDGETCAMERAFOCUSCONTROLLER_H

/// @file OsgWidgetCameraFocusController.h
/// @brief 根据后端 id 合并包围球并设置轨道相机（从 OsgWidget 拆出）。

#include <string>

class OsgWidget;

/// 根据后端 id 合并包围球并设置轨道相机（从 OsgWidget 拆出）。
class OsgWidgetCameraFocusController
{
public:
	static void focusCameraOnBackend(OsgWidget& self, const std::string& backendId);
};

#endif // CLOUDSIMHOST_OSGWIDGETCAMERAFOCUSCONTROLLER_H
