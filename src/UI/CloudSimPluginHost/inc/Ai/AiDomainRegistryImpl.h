#ifndef CLOUDSIMPLUGINHOST_AIDOMAINREGISTRYIMPL_H
#define CLOUDSIMPLUGINHOST_AIDOMAINREGISTRYIMPL_H

/// @file AiDomainRegistryImpl.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AiDomainRegistryImpl 接口

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

#endif // CLOUDSIMPLUGINHOST_AIDOMAINREGISTRYIMPL_H
