#ifndef CLOUDSIMHOST_HOSTOSGFLAVORHOOKS_H
#define CLOUDSIMHOST_HOSTOSGFLAVORHOOKS_H

/// @file HostOsgFlavorHooks.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 桌面/Headless 各自实现，消除共享源 CLOUDSIM_HOST_HEADLESS_ONLY

#include "IRenderView.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

class BackendDataManager;
class IOsgWidgetView;
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
	QWidget* osgPane = nullptr;
	std::unique_ptr<core::IRenderView> renderView;
};

/// 桌面 true；Headless false（由各 DLL 提供定义）
bool hostOsgFlavorEnabled();

std::unique_ptr<core::IRenderView> createFlavorRenderView(QWidget* parent);
std::unique_ptr<core::IRenderViewFactory> createFlavorRenderViewFactory();
std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget);
std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget, DocumentHost& host);

FlavorOsgViewportMount mountFlavorOsgViewport(DocumentHost& host, QVBoxLayout& centralLayout,
											  BackendDataManager& backend, FlavorVisualSyncMarkDirtyFn markDirty);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HOSTOSGFLAVORHOOKS_H
