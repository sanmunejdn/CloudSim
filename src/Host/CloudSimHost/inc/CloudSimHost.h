#pragma once

#include "cloudsim_host_global.h"

#include <memory>

class QWidget;

namespace cloudsim::core {
class EventHub;
class IDocumentScope;
class IRenderViewFactory;
} // namespace cloudsim::core

namespace cloudsim::host {

class DocumentHost;

/// 创建单文档宿主（ApplicationContextImpl::createDocumentScope 调用）
CLOUDSIM_HOST_EXPORT std::unique_ptr<core::IDocumentScope> createDocumentHost(QWidget* parent, core::EventHub& events,
	const QString& documentId);

/// 创建 Host 侧 IRenderViewFactory
CLOUDSIM_HOST_EXPORT std::unique_ptr<core::IRenderViewFactory> createHostRenderViewFactory();

/// IDocumentScope → DocumentHost；失败返回 nullptr
CLOUDSIM_HOST_EXPORT DocumentHost* documentHostFromScope(core::IDocumentScope* scope);

} // namespace cloudsim::host

/// C ABI：校验 cloudsimCoreApiVersion 后返回渲染工厂，供动态加载方
extern "C" CLOUDSIM_HOST_EXPORT cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion);
