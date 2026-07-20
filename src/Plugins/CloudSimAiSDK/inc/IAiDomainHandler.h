#ifndef CLOUDSIMAISDK_IAIDOMAINHANDLER_H
#define CLOUDSIMAISDK_IAIDOMAINHANDLER_H

/// @file IAiDomainHandler.h
/// @brief IAiDomainHandler 接口

#include "cloudsim_ai_sdk_global.h"

#include "AiDomainTypes.h"

#include <QByteArray>
#include <QString>

class IPluginHostContext;
class IAiAssistantHost;

class IAiDomainHandler
{
public:
	virtual ~IAiDomainHandler() = default;

	virtual QString domainId() const = 0;
	virtual bool validateOutput(const QByteArray& jsonUtf8, QString* err) const = 0;
	virtual bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost,
						 QString* summary, QString* err) = 0;

	virtual bool adaptToActionPlan(const QByteArray& domainJson, QByteArray* outPlan, QString* err) const
	{
		(void)domainJson;
		(void)outPlan;
		if (err)
			*err = QStringLiteral("adaptToActionPlan not implemented.");
		return false;
	}
};

class IAiDomainRegistry
{
public:
	virtual ~IAiDomainRegistry() = default;

	virtual bool registerDomain(const AiDomainDescriptor& desc, IAiDomainHandler* handler) = 0;
	virtual QStringList listDomainIds() const = 0;
	virtual const AiDomainDescriptor* descriptor(const QString& domainId) const = 0;
	virtual IAiDomainHandler* handler(const QString& domainId) = 0;
};

#endif // CLOUDSIMAISDK_IAIDOMAINHANDLER_H
