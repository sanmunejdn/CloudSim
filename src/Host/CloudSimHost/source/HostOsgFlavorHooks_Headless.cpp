/// @file HostOsgFlavorHooks_Headless.cpp
/// @brief Headless：无 OSG，一律 Null 渲染

#include "HostOsgFlavorHooks.h"

#include "BackendDataManager.h"
#include "DocumentHost.h"
#include "NullCoreServices.h"

#include <QString>
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

bool flavorSetMultiViewLayout(QWidget* osgPane, MultiViewLayout layout)
{
	(void)osgPane;
	(void)layout;
	return false;
}

MultiViewLayout flavorMultiViewLayout(QWidget* osgPane)
{
	(void)osgPane;
	return MultiViewLayout::Single;
}

int flavorActiveViewIndex(QWidget* osgPane)
{
	(void)osgPane;
	return 0;
}

bool flavorOpenDetachedViewWindow(QWidget* osgPane)
{
	(void)osgPane;
	return false;
}

void flavorCloseDetachedViewWindows(QWidget* osgPane)
{
	(void)osgPane;
}

void flavorPersistMultiViewSettings(QWidget* osgPane, const QString& documentId)
{
	(void)osgPane;
	(void)documentId;
}

void flavorRestoreMultiViewSettings(QWidget* osgPane, const QString& documentId)
{
	(void)osgPane;
	(void)documentId;
}

void flavorSetMultiViewUseChinese(QWidget* osgPane, bool useChinese)
{
	(void)osgPane;
	(void)useChinese;
}

} // namespace cloudsim::host
