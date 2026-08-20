#ifndef CLOUDSIMPLUGINHOST_GEOMETRYRECOGNIZEDOMAINHANDLER_H
#define CLOUDSIMPLUGINHOST_GEOMETRYRECOGNIZEDOMAINHANDLER_H

/// @file GeometryRecognizeDomainHandler.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief GeometryRecognizeDomainHandler 接口

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

#endif // CLOUDSIMPLUGINHOST_GEOMETRYRECOGNIZEDOMAINHANDLER_H
