#ifndef CLOUDSIMPLUGINHOST_AIDOMAINROUTER_H
#define CLOUDSIMPLUGINHOST_AIDOMAINROUTER_H

/// @file AiDomainRouter.h
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
