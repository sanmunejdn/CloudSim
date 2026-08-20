#ifndef CLOUDSIMPLUGINHOST_MESHCOMPOSEDOMAINHANDLER_H
#define CLOUDSIMPLUGINHOST_MESHCOMPOSEDOMAINHANDLER_H

/// @file MeshComposeDomainHandler.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief MeshComposeDomainHandler 接口

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

#endif // CLOUDSIMPLUGINHOST_MESHCOMPOSEDOMAINHANDLER_H
