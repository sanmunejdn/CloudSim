#ifndef CLOUDSIMPLUGINHOST_DESIGNPARTSDOMAINHANDLER_H
#define CLOUDSIMPLUGINHOST_DESIGNPARTSDOMAINHANDLER_H

/// @file DesignPartsDomainHandler.h
/// @brief design.parts：标准件检索实例化 → feature.compose 执行

#include "Ai/DesignPartsCatalog.h"
#include "IAiDomainHandler.h"

class DesignPartsDomainHandler : public IAiDomainHandler
{
public:
	DesignPartsDomainHandler();

	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
				 QString* err) override;

	bool tryParseUserText(const QString& text, QByteArray* outPlanUtf8, QString* hint, QString* err) const;

	const DesignPartsCatalog& catalog() const { return m_catalog; }

private:
	DesignPartsCatalog m_catalog;
};

#endif
