#pragma once

#include "IAiDomainHandler.h"

#include <json.hpp>

class MeshComposeDomainHandler : public IAiDomainHandler
{
public:
	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
		QString* err) override;

	static bool validatePlanJson(const nlohmann::json& root, QString* err);
};
