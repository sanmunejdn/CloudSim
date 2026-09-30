#ifndef CLOUDSIMHOST_HOSTOSGFLAVORHOOKS_H
#define CLOUDSIMHOST_HOSTOSGFLAVORHOOKS_H

/// @file HostOsgFlavorHooks.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 桌面/Headless 各自实现，消除共享源 CLOUDSIM_HOST_HEADLESS_ONLY

#include "IRenderView.h"
#include "cloudsim_viewport_global.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

class BackendDataManager;
class IOsgWidgetView;
class IViewportSceneSignals;
class OsgWidget;
class QVBoxLayout;
class QWidget;

namespace cloudsim::host
{
class DocumentHost;

using FlavorVisualSyncMarkDirtyFn = std::function<void(const std::string&, std::uint32_t)>;

struct FlavorOsgViewportMount
{
	OsgWidget* osgWidget = nullptr;
	IOsgWidgetView* osgView = nullptr;
	IViewportSceneSignals* viewportSceneSignals = nullptr;
	QWidget* osgPane = nullptr;
	std::unique_ptr<core::IRenderView> renderView;
};

// 桌面实现在 Viewport.dll；Headless 同 DLL 本地定义（声明不加 dllimport）
#if defined(CLOUDSIM_HOST_HEADLESS_ONLY)
#define HOST_OSG_FLAVOR_API
#else
#define HOST_OSG_FLAVOR_API CLOUDSIM_VIEWPORT_EXPORT
#endif

/// 桌面 true；Headless false（由各 DLL 提供定义）
HOST_OSG_FLAVOR_API bool hostOsgFlavorEnabled();

HOST_OSG_FLAVOR_API std::unique_ptr<core::IRenderView> createFlavorRenderView(QWidget* parent);
HOST_OSG_FLAVOR_API std::unique_ptr<core::IRenderViewFactory> createFlavorRenderViewFactory();
HOST_OSG_FLAVOR_API std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget);
HOST_OSG_FLAVOR_API std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget, DocumentHost& host);

HOST_OSG_FLAVOR_API FlavorOsgViewportMount mountFlavorOsgViewport(DocumentHost& host, QVBoxLayout& centralLayout,
											  BackendDataManager& backend, FlavorVisualSyncMarkDirtyFn markDirty);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HOSTOSGFLAVORHOOKS_H
