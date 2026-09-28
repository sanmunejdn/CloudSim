/// @file AiDomainRouter.cpp
/// @brief AiDomainRouter：委托规则打分意图分类

#include "Ai/AiDomainRouter.h"

#include "Ai/AiIntentClassifier.h"
#include "AiDomainTypes.h"

AiDomainRouter::AiDomainRouter(const IAiDomainRegistry* registry) : m_registry(registry) {}

QString AiDomainRouter::resolve(const QString& requestedDomainId, const QString& userText) const
{
	const QString req = requestedDomainId.trimmed();
	if (!req.isEmpty() && req != AiDomainIds::autoDomain())
		return req;

	const auto r = AiIntentClassifier::classifyByRules(userText, 2);
	return r.domainId;
}
