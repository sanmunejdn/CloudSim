#ifndef CLOUDSIMPLUGINHOST_CATALOGACTIONPLANDOMAINHANDLER_H
#define CLOUDSIMPLUGINHOST_CATALOGACTIONPLANDOMAINHANDLER_H

/// @file CatalogActionPlanDomainHandler.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 通用 ActionPlan 域：校验 steps.api 属于本域后执行

#include "IAiDomainHandler.h"

#include <QByteArray>
#include <QString>

class CatalogActionPlanDomainHandler : public IAiDomainHandler
{
public:
	CatalogActionPlanDomainHandler(QString domainId, QByteArray catalogJsonUtf8);

	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
				 QString* err) override;

private:
	QString m_domainId;
	QByteArray m_catalogJsonUtf8;
};

#endif
