/// @file HostRenderViewFactory.cpp
/// @brief HostRenderView 工厂

#include "HostRenderViewFactory.h"

#include "NullCoreServices.h"

#ifndef CLOUDSIM_HOST_HEADLESS_ONLY

#include "OsgWidget.h"
#include "adapters/OsgRenderViewAdapter.h"

#include <QVBoxLayout>

namespace cloudsim::host
{
std::unique_ptr<core::IRenderView> wrapOsgWidgetAsRenderView(OsgWidget& widget)
{
	return std::make_unique<OsgRenderViewAdapter>(widget);
}

std::unique_ptr<core::IRenderView> HostRenderViewFactory::createView(QWidget* parent)
{
	auto* container = new QWidget(parent);
	auto* layout = new QVBoxLayout(container);
	layout->setContentsMargins(0, 0, 0, 0);
	auto* osg = new OsgWidget(container);
	layout->addWidget(osg);
	return std::make_unique<OsgRenderViewAdapter>(*osg);
}

} // namespace cloudsim::host

#else

namespace cloudsim::host
{
std::unique_ptr<core::IRenderView> wrapOsgWidgetAsRenderView(OsgWidget& widget)
{
	(void)widget;
	return core::makeNullRenderViewFactory()->createView(nullptr);
}

std::unique_ptr<core::IRenderView> HostRenderViewFactory::createView(QWidget* parent)
{
	return core::makeNullRenderViewFactory()->createView(parent);
}

} // namespace cloudsim::host

#endif
