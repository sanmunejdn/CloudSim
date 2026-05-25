#pragma once

#include <osg/Vec4>
#include <string>

class OsgWidget;

/// 将显示颜色应用到暂存几何或后端对象分支（从 OsgWidget 拆出，自 OsgWidget 拆出
class OsgWidgetColorController
{
public:
	static void applyColorToStagingGeometry(OsgWidget& self, const osg::Vec4& color);
	static void applyColorToBackendObject(OsgWidget& self, const std::string& backendId, const osg::Vec4& color);
	static void applyColorToActiveBackendObject(OsgWidget& self, const osg::Vec4& color);
};
