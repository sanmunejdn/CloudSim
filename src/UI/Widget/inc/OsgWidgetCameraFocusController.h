#pragma once

#include <string>

class OsgWidget;

/// 根据后端 id 合并包围球并设置轨道相机（从 OsgWidget 拆出）。
class OsgWidgetCameraFocusController
{
public:
	static void focusCameraOnBackend(OsgWidget& self, const std::string& backendId);
};
