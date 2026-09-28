/// @file HostOsgFlavorHooks_Headless.cpp
/// @brief Headless：无 OSG，一律 Null 渲染

#include "HostOsgFlavorHooks.h"

#include "BackendDataManager.h"
#include "DocumentHost.h"
#include "NullCoreServices.h"

#include <QVBoxLayout>

namespace cloudsim::host
{
bool hostOsgFlavorEnabled()
{
	return false;
}

std::unique_ptr<core::IRenderView> createFlavorRenderView(QWidget* parent)
{
	return core::makeNullRenderViewFactory()->createView(parent);
}

std::unique_ptr<core::IRenderViewFactory> createFlavorRenderViewFactory()
{
	return core::makeNullRenderViewFactory();
}

std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget)
{
	(void)widget;
	return core::makeNullRenderViewFactory()->createView(nullptr);
}

std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget, DocumentHost& host)
{
	(void)widget;
	(void)host;
	return core::makeNullRenderViewFactory()->createView(nullptr);
}

FlavorOsgViewportMount mountFlavorOsgViewport(DocumentHost& host, QVBoxLayout& centralLayout,
											  BackendDataManager& backend, FlavorVisualSyncMarkDirtyFn markDirty)
{
	(void)host;
	(void)centralLayout;
	(void)backend;
	(void)markDirty;
	return {};
}

} // namespace cloudsim::host
