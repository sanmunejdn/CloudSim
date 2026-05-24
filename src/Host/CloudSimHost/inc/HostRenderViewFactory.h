#pragma once

#include "cloudsim_host_global.h"
#include "IRenderView.h"

class OsgWidget;

namespace cloudsim::host {

/// 按需创建独立 OsgWidget 视口并包装为 IRenderView；仅工厂职责，不做事件编排
class CLOUDSIM_HOST_EXPORT HostRenderViewFactory final : public core::IRenderViewFactory
{
public:
	std::unique_ptr<core::IRenderView> createView(QWidget* parent) override;
};

/// DocumentHost 内已有 OsgWidget 时复用包装，避免重复创建视口
std::unique_ptr<core::IRenderView> wrapOsgWidgetAsRenderView(OsgWidget& widget);

} // namespace cloudsim::host
