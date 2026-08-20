#ifndef WIDGET_OSGWIDGETCAMERAFOCUSCONTROLLER_H
#define WIDGET_OSGWIDGETCAMERAFOCUSCONTROLLER_H

/// @file OsgWidgetCameraFocusController.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 根据后端 id 合并包围球并设置轨道相机（自 OsgWidget 拆出

#include <string>

class OsgWidget;

/// 根据后端 id 合并包围球并设置轨道相机（自 OsgWidget 拆出
class OsgWidgetCameraFocusController
{
public:
	static void focusCameraOnBackend(OsgWidget& self, const std::string& backendId);
};

#endif // WIDGET_OSGWIDGETCAMERAFOCUSCONTROLLER_H
