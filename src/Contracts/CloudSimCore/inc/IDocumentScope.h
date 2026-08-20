#ifndef CLOUDSIMCORE_IDOCUMENTSCOPE_H
#define CLOUDSIMCORE_IDOCUMENTSCOPE_H

/// @file IDocumentScope.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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
