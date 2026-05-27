#pragma once

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
