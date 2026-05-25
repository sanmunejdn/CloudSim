#pragma once

#include "cloudsim_host_global.h"
#include "IRenderView.h"

class OsgWidget;

namespace cloudsim::host {

/// Host 渲染工厂
class CLOUDSIM_HOST_EXPORT HostRenderViewFactory final : public core::IRenderViewFactory
{
public:
	std::unique_ptr<core::IRenderView> createView(QWidget* parent) override;
};

/// 包装已有 OsgWidget
std::unique_ptr<core::IRenderView> wrapOsgWidgetAsRenderView(OsgWidget& widget);

} // namespace cloudsim::host
