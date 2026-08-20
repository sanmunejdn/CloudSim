#ifndef CLOUDSIMPLUGINHOST_AIDOMAINROUTER_H
#define CLOUDSIMPLUGINHOST_AIDOMAINROUTER_H

/// @file AiDomainRouter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AiDomainRouter 接口

#include "IAiDomainHandler.h"

#include <QString>

class AiDomainRouter
{
public:
	explicit AiDomainRouter(const IAiDomainRegistry* registry);

	QString resolve(const QString& requestedDomainId, const QString& userText) const;

private:
	const IAiDomainRegistry* m_registry = nullptr;
};

#endif // CLOUDSIMPLUGINHOST_AIDOMAINROUTER_H
