#pragma once

#include "IAiDomainHandler.h"

#include <QHash>
#include <memory>

class AiDomainRegistryImpl : public IAiDomainRegistry
{
public:
	bool registerDomain(const AiDomainDescriptor& desc, IAiDomainHandler* handler) override;
	QStringList listDomainIds() const override;
	const AiDomainDescriptor* descriptor(const QString& domainId) const override;
	IAiDomainHandler* handler(const QString& domainId) override;

private:
	struct Entry
	{
		AiDomainDescriptor desc;
		IAiDomainHandler* handler = nullptr;
	};
	QHash<QString, Entry> m_entries;
};
