#ifndef CLOUDSIMPLUGINHOST_FEATURECOMPOSEDOMAINHANDLER_H
#define CLOUDSIMPLUGINHOST_FEATURECOMPOSEDOMAINHANDLER_H

/// @file FeatureComposeDomainHandler.h
/// @brief feature.compose：Parametric 特征链 ActionPlan

#include "IAiDomainHandler.h"

#include <json.hpp>

class FeatureComposeDomainHandler : public IAiDomainHandler
{
public:
	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
				 QString* err) override;

	static bool validatePlanJson(const nlohmann::json& root, QString* err);
};

#endif
