#ifndef WIDGET_OSGWIDGETCOLORCONTROLLER_H
#define WIDGET_OSGWIDGETCOLORCONTROLLER_H

/// @file OsgWidgetColorController.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 将显示颜色应用到暂存几何或后端对象分支（从 OsgWidget 拆出，自 OsgWidget 拆出

#include <string>

#include <osg/Vec4>

class OsgWidget;

/// 将显示颜色应用到暂存几何或后端对象分支（从 OsgWidget 拆出，自 OsgWidget 拆出
class OsgWidgetColorController
{
public:
	static void applyColorToStagingGeometry(OsgWidget& self, const osg::Vec4& color);
	static void applyColorToBackendObject(OsgWidget& self, const std::string& backendId, const osg::Vec4& color);
	static void applyColorToActiveBackendObject(OsgWidget& self, const osg::Vec4& color);
};

#endif // WIDGET_OSGWIDGETCOLORCONTROLLER_H
