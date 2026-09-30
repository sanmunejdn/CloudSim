#ifndef CLOUDSIMHOST_OSGWIDGETSCENEBRIDGE_H
#define CLOUDSIMHOST_OSGWIDGETSCENEBRIDGE_H

/// @file OsgWidgetSceneBridge.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IBackendSceneBridge 委托至 OsgWidget；widget 未绑定时操作为空操作

#include "widget_global.h"

#include "IBackendSceneBridge.h"

class OsgWidget;

// 桌面实现在 Viewport；Headless 实现在 Headless DLL；Core 调用方按宏选 import/export
#if defined(CLOUDSIM_VIEWPORT_LIB)
#define OSG_SCENE_BRIDGE_API CLOUDSIM_VIEWPORT_EXPORT
#elif defined(CLOUDSIM_HOST_HEADLESS_ONLY)
#define OSG_SCENE_BRIDGE_API CLOUDSIM_HOST_EXPORT
#else
#define OSG_SCENE_BRIDGE_API CLOUDSIM_VIEWPORT_EXPORT
#endif

/// IBackendSceneBridge 委托至 OsgWidget；widget 未绑定时操作为空操作
class OSG_SCENE_BRIDGE_API OsgWidgetSceneBridge final : public IBackendSceneBridge
{
public:
	OsgWidgetSceneBridge() = default;
	explicit OsgWidgetSceneBridge(OsgWidget* widget) : m_widget(widget) {}

	void setOsgWidget(OsgWidget* widget) { m_widget = widget; }
	OsgWidget* osgWidget() const { return m_widget; }

	void setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
											  const std::array<double, 16>& columnMajor4x4) override;
	bool getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
											  std::array<double, 16>& outColumnMajor4x4) const override;

	void setBackendObjectVisible(const std::string& backendId, bool visible) override;
	void removeBackendObjectVisual(const std::string& backendId) override;
	bool hasBackendObjectBranch(const std::string& backendId) const override;

	bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
									double& outCz) const override;

	void syncOuterPatFromBackend(const BackendDataBase& data) override;
	void setBackendParent(const std::string& backendId, const std::string& parentBackendId) override;

private:
	OsgWidget* m_widget = nullptr;
};

#endif // CLOUDSIMHOST_OSGWIDGETSCENEBRIDGE_H
