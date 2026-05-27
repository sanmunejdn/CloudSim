#pragma once

#include "AiConfigDto.h"
#include "AiInferenceTypes.h"
#include "AiParseTypes.h"
#include "cloudsim_ai_sdk_global.h"

#include <functional>
#include <memory>
#include <optional>

class IAiDomainRegistry;
class IAiInferenceProvider;

class IAiAssistantHost
{
public:
	virtual ~IAiAssistantHost() = default;

	virtual unsigned int aiSdkVersion() const = 0;

	virtual std::optional<AiConfigDto> loadConfig() const = 0;
	virtual bool saveConfig(const AiConfigDto& config, QString* errorMessage = nullptr) const = 0;

	virtual QByteArray apiCatalogJson() const = 0;
	virtual IAiDomainRegistry* domainRegistry() = 0;
	virtual const IAiDomainRegistry* domainRegistry() const = 0;

	virtual void registerInferenceProvider(std::shared_ptr<IAiInferenceProvider> provider) = 0;

	virtual AiParseResult parseUserTextWithRules(const QString& domainId, const QString& text) const = 0;

	virtual void parseUserTextAsync(const AiInferenceRequest& request, const AiConfigDto& config,
		const AiInferenceProgressFn& progress, std::function<void(AiParseResult)> onFinished) = 0;

	virtual bool executeActionPlan(const QByteArray& actionPlanJsonUtf8, QString* outSummary, QString* outError) = 0;

	virtual bool executeDomainOutput(const QString& domainId, const QByteArray& outputJsonUtf8, QString* outSummary,
		QString* outError) = 0;

	virtual QString resolveDomainId(const QString& requestedDomainId, const QString& userText) const = 0;
};
