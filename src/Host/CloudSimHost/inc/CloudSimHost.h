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

/// 创建文档宿主
CLOUDSIM_HOST_EXPORT std::unique_ptr<core::IDocumentScope> createDocumentHost(QWidget* parent, core::EventHub& events,
	const QString& documentId);

/// 创建渲染工厂
CLOUDSIM_HOST_EXPORT std::unique_ptr<core::IRenderViewFactory> createHostRenderViewFactory();

/// 作用域转 Host
CLOUDSIM_HOST_EXPORT DocumentHost* documentHostFromScope(core::IDocumentScope* scope);

} // namespace cloudsim::host

/// C ABI 渲染工厂
extern "C" CLOUDSIM_HOST_EXPORT cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion);
