#ifndef CLOUDSIMCORE_IDOCUMENTSCOPE_H
#define CLOUDSIMCORE_IDOCUMENTSCOPE_H

/// @file IDocumentScope.h
/// @brief 单文档作用域

#include "cloudsim_core_global.h"

namespace cloudsim::core
{
class IDataService;
class IRobotService;
class IRenderView;

/// 单文档作用域
class CLOUDSIM_CORE_EXPORT IDocumentScope
{
public:
	virtual ~IDocumentScope();

	virtual QString documentId() const = 0;
	virtual IDataService& data() = 0;
	virtual IRobotService& robot() = 0;
	virtual IRenderView& render() = 0;
};

} // namespace cloudsim::core

#endif // CLOUDSIMCORE_IDOCUMENTSCOPE_H
