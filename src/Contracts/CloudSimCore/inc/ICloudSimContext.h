#pragma once

#include "cloudsim_core_global.h"

#include <memory>

class QWidget;

namespace cloudsim::core {

class EventHub;
class IDocumentScope;
class IRenderViewFactory;

/// 应用组合根
class CLOUDSIM_CORE_EXPORT ICloudSimContext
{
public:
	virtual ~ICloudSimContext();

	virtual EventHub& events() = 0;
	virtual IRenderViewFactory& renderFactory() = 0;

	virtual std::unique_ptr<IDocumentScope> createDocumentScope(QWidget* parent, const QString& documentId) = 0;

	virtual IDocumentScope* activeScope() const = 0;
	virtual void setActiveScope(IDocumentScope* scope) = 0;
};

} // namespace cloudsim::core
