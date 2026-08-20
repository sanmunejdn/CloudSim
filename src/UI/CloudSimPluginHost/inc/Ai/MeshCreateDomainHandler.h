#ifndef CLOUDSIMPLUGINHOST_MESHCREATEDOMAINHANDLER_H
#define CLOUDSIMPLUGINHOST_MESHCREATEDOMAINHANDLER_H

/// @file MeshCreateDomainHandler.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief MeshCreateDomainHandler 接口

#include "IAiDomainHandler.h"

class MeshCreateDomainHandler : public IAiDomainHandler
{
public:
	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
				 QString* err) override;
};

#endif // CLOUDSIMPLUGINHOST_MESHCREATEDOMAINHANDLER_H
