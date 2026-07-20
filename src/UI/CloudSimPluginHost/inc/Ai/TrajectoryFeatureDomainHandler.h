#ifndef CLOUDSIMPLUGINHOST_TRAJECTORYFEATUREDOMAINHANDLER_H
#define CLOUDSIMPLUGINHOST_TRAJECTORYFEATUREDOMAINHANDLER_H

/// @file TrajectoryFeatureDomainHandler.h
/// @brief TrajectoryFeatureDomainHandler 接口

#include "IAiDomainHandler.h"

class TrajectoryFeatureDomainHandler : public IAiDomainHandler
{
public:
	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
				 QString* err) override;
	bool adaptToActionPlan(const QByteArray& domainJson, QByteArray* outPlan, QString* err) const override;
};

#endif // CLOUDSIMPLUGINHOST_TRAJECTORYFEATUREDOMAINHANDLER_H
