/// @file HostOsgFlavorHooks_Desktop.cpp
/// @brief 桌面：真 OSG 视口装配

#include "HostOsgFlavorHooks.h"

#include "BackendDataManager.h"
#include "DocumentHost.h"
#include "HostRenderViewFactory.h"
#include "OsgWidget.h"
#include "adapters/OsgRenderViewAdapter.h"

#include <QVBoxLayout>

namespace cloudsim::host
{
bool hostOsgFlavorEnabled()
{
	return true;
}

std::unique_ptr<core::IRenderView> createFlavorRenderView(QWidget* parent)
{
	auto* container = new QWidget(parent);
	auto* layout = new QVBoxLayout(container);
	layout->setContentsMargins(0, 0, 0, 0);
	auto* osg = new OsgWidget(container);
	layout->addWidget(osg);
	return std::make_unique<OsgRenderViewAdapter>(*osg);
}

std::unique_ptr<core::IRenderViewFactory> createFlavorRenderViewFactory()
{
	return std::make_unique<HostRenderViewFactory>();
}

std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget)
{
	return std::make_unique<OsgRenderViewAdapter>(widget);
}

std::unique_ptr<core::IRenderView> wrapFlavorOsgWidget(OsgWidget& widget, DocumentHost& host)
{
	return std::make_unique<OsgRenderViewAdapter>(widget, host);
}

FlavorOsgViewportMount mountFlavorOsgViewport(DocumentHost& host, QVBoxLayout& centralLayout,
											  BackendDataManager& backend, FlavorVisualSyncMarkDirtyFn markDirty)
{
	FlavorOsgViewportMount out;
	// OSG 直挂 layout：勿用 QStackedWidget 包 OpenGL，Windows 上会拖视图卡顿
	auto* osg = new OsgWidget(&host);
	osg->setPoseSyncBackendManager(&backend);
	osg->setVisualSyncMarkDirty(std::move(markDirty));
	host.sceneBridge().setOsgWidget(osg);
	centralLayout.addWidget(osg);
	out.osgWidget = osg;
	out.osgView = osg;
	out.viewportSceneSignals = osg;
	out.osgPane = osg;
	out.renderView = wrapFlavorOsgWidget(*osg, host);
	return out;
}

} // namespace cloudsim::host
