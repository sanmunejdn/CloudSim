#pragma once

#include "cloudsim_host_global.h"
#include "IRenderView.h"

class OsgWidget;

namespace cloudsim::host {

class CLOUDSIM_HOST_EXPORT HostRenderViewFactory final : public core::IRenderViewFactory
{
public:
	std::unique_ptr<core::IRenderView> createView(QWidget* parent) override;
};

/// 将已创建的 OsgWidget 包装为 IRenderView（文档宿主内部使用）。
std::unique_ptr<core::IRenderView> wrapOsgWidgetAsRenderView(OsgWidget& widget);

} // namespace cloudsim::host
