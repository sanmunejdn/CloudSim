#ifndef POINTNETPLUGIN_POINTNETDOMAINHANDLER_H
#define POINTNETPLUGIN_POINTNETDOMAINHANDLER_H

/// @file PointNetDomainHandler.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PointNet++ 分类域处理器

#include "IAiDomainHandler.h"

class PointNetInference;

/// PointNet++ 分类域处理器
class PointNetClassifyDomainHandler : public IAiDomainHandler
{
public:
	explicit PointNetClassifyDomainHandler(PointNetInference* inference);

	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
				 QString* err) override;

private:
	PointNetInference* m_inference;
};

/// PointNet++ 分割域处理器
class PointNetSegmentDomainHandler : public IAiDomainHandler
{
public:
	explicit PointNetSegmentDomainHandler(PointNetInference* inference);

	QString domainId() const override;
	bool validateOutput(const QByteArray& jsonUtf8, QString* err) const override;
	bool execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost, QString* summary,
				 QString* err) override;

private:
	PointNetInference* m_inference;
};

#endif // POINTNETPLUGIN_POINTNETDOMAINHANDLER_H
