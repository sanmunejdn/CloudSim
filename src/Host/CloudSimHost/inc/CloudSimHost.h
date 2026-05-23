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

CLOUDSIM_HOST_EXPORT std::unique_ptr<core::IDocumentScope> createDocumentHost(QWidget* parent, core::EventHub& events,
	const QString& documentId);

CLOUDSIM_HOST_EXPORT std::unique_ptr<core::IRenderViewFactory> createHostRenderViewFactory();

CLOUDSIM_HOST_EXPORT DocumentHost* documentHostFromScope(core::IDocumentScope* scope);

} // namespace cloudsim::host

extern "C" CLOUDSIM_HOST_EXPORT cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion);
