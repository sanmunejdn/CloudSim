#include "HostRenderViewFactory.h"

#include "adapters/OsgRenderViewAdapter.h"
#include "OsgWidget.h"

#include <QVBoxLayout>

namespace cloudsim::host {

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
