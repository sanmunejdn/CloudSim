#ifndef CLOUDSIMCORE_ICLOUDSIMCONTEXT_H
#define CLOUDSIMCORE_ICLOUDSIMCONTEXT_H

/// @file ICloudSimContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 应用组合根

#include "cloudsim_core_global.h"

#include <memory>

class QWidget;

namespace cloudsim::core
{
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

#endif // CLOUDSIMCORE_ICLOUDSIMCONTEXT_H
