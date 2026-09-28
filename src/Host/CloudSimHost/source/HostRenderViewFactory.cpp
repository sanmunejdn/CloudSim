/// @file HostRenderViewFactory.cpp
/// @brief HostRenderView 工厂（经 flavor hooks，无 HEADLESS 宏）

#include "HostRenderViewFactory.h"

#include "HostOsgFlavorHooks.h"

namespace cloudsim::host
{
std::unique_ptr<core::IRenderView> wrapOsgWidgetAsRenderView(OsgWidget& widget)
{
	return wrapFlavorOsgWidget(widget);
}

std::unique_ptr<core::IRenderView> HostRenderViewFactory::createView(QWidget* parent)
{
	return createFlavorRenderView(parent);
}

} // namespace cloudsim::host
