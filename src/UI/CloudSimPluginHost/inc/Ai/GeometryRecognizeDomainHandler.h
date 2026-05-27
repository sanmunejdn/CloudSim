#pragma once

#include "IAiDomainHandler.h"

class GeometryRecognizeDomainHandler : public IAiDomainHandler
{
public:
	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
		QString* err) override;
	bool adaptToActionPlan(const QByteArray& domainJson, QByteArray* outPlan, QString* err) const override;
};
