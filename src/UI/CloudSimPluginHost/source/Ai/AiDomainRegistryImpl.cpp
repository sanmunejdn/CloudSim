/// @file AiDomainRegistryImpl.cpp
/// @brief AiDomainRegistryImpl 实现

#include "Ai/AiDomainRegistryImpl.h"

bool AiDomainRegistryImpl::registerDomain(const AiDomainDescriptor& desc, IAiDomainHandler* handler)
{
	if (desc.domainId.isEmpty() || !handler)
		return false;
	Entry e;
	e.desc = desc;
	e.handler = handler;
	m_entries.insert(desc.domainId, e);
	return true;
}

QStringList AiDomainRegistryImpl::listDomainIds() const
{
	return m_entries.keys();
}

const AiDomainDescriptor* AiDomainRegistryImpl::descriptor(const QString& domainId) const
{
	const auto it = m_entries.constFind(domainId);
	if (it == m_entries.constEnd())
		return nullptr;
	return &it->desc;
}

IAiDomainHandler* AiDomainRegistryImpl::handler(const QString& domainId)
{
	const auto it = m_entries.find(domainId);
	if (it == m_entries.end())
		return nullptr;
	return it->handler;
}
