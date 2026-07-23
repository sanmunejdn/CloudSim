#ifndef CLOUDSIMAISDK_IAIASSISTANTHOST_H
#define CLOUDSIMAISDK_IAIASSISTANTHOST_H

/// @file IAiAssistantHost.h
/// @brief IAiAssistantHost 接口

#include "cloudsim_ai_sdk_global.h"

#include "AiConfigDto.h"
#include "AiInferenceTypes.h"
#include "AiParseTypes.h"
#include "AiAgentTypes.h"

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

	virtual AiParseResult parseTrajectoryFeatureRequest(const AiInferenceRequest& request) const = 0;

	virtual void parseUserTextAsync(const AiInferenceRequest& request, const AiConfigDto& config,
									const AiInferenceProgressFn& progress,
									std::function<void(AiParseResult)> onFinished) = 0;

	virtual bool executeActionPlan(const QByteArray& actionPlanJsonUtf8, QString* outSummary, QString* outError) = 0;

	virtual bool executeDomainOutput(const QString& domainId, const QByteArray& outputJsonUtf8, QString* outSummary,
									 QString* outError) = 0;

	virtual QString resolveDomainId(const QString& requestedDomainId, const QString& userText) const = 0;

	/// Agent 回合（vtable 末尾追加）
	virtual void runAgentTurnAsync(const AiInferenceRequest& request, const AiConfigDto& config,
								   const AiInferenceProgressFn& progress, const AiAgentEventFn& onEvent) = 0;
	virtual void submitAgentConfirm(const QString& pendingId, const QByteArray& argsJsonUtf8) = 0;
	virtual void cancelAgentConfirm(const QString& pendingId) = 0;
	virtual void cancelAgentTurn() = 0;
	/// 识别/轨迹等域确认并入同一 Session
	virtual void beginDomainConfirmAsync(const AiDomainConfirmRequest& request, const AiAgentEventFn& onEvent) = 0;
	virtual void secondaryAgentConfirm(const QString& pendingId) = 0;
};

#endif // CLOUDSIMAISDK_IAIASSISTANTHOST_H
