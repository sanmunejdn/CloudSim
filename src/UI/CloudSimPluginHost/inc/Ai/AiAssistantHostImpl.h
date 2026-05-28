#pragma once

#include "IAiAssistantHost.h"

#include "Ai/AiDomainRegistryImpl.h"
#include "Ai/AiDomainRouter.h"
#include "Ai/GeometryRecognizeDomainHandler.h"
#include "Ai/MeshComposeDomainHandler.h"
#include "Ai/MeshCreateDomainHandler.h"

#include <memory>

class PluginHostContext;

class AiAssistantHostImpl : public IAiAssistantHost
{
public:
	explicit AiAssistantHostImpl(PluginHostContext* pluginHost);

	unsigned int aiSdkVersion() const override;
	std::optional<AiConfigDto> loadConfig() const override;
	bool saveConfig(const AiConfigDto& config, QString* errorMessage = nullptr) const override;

	QByteArray apiCatalogJson() const override;
	IAiDomainRegistry* domainRegistry() override;
	const IAiDomainRegistry* domainRegistry() const override;

	void registerInferenceProvider(std::shared_ptr<IAiInferenceProvider> provider) override;

	AiParseResult parseUserTextWithRules(const QString& domainId, const QString& text) const override;

	void parseUserTextAsync(const AiInferenceRequest& request, const AiConfigDto& config,
		const AiInferenceProgressFn& progress, std::function<void(AiParseResult)> onFinished) override;

	bool executeActionPlan(const QByteArray& actionPlanJsonUtf8, QString* outSummary, QString* outError) override;

	bool executeDomainOutput(const QString& domainId, const QByteArray& outputJsonUtf8, QString* outSummary,
		QString* outError) override;

	QString resolveDomainId(const QString& requestedDomainId, const QString& userText) const override;

private:
	void registerBuiltinDomains();
	const AiDomainModelConfig* findDomainConfig(const AiConfigDto& cfg, const QString& domainId) const;
	AiParseResult parseWithLocalLlm(const QString& domainId, const QString& text, const AiDomainModelConfig& dm,
		const QByteArray& imagePng, const AiInferenceProgressFn& progress) const;
	AiParseResult parseWithRemoteLlm(const QString& domainId, const QString& text, const AiRemoteLlmConfig& remote,
		const AiInferenceProgressFn& progress) const;

	PluginHostContext* m_pluginHost = nullptr;
	AiDomainRegistryImpl m_registry;
	AiDomainRouter m_router;
	MeshCreateDomainHandler m_meshHandler;
	MeshComposeDomainHandler m_composeHandler;
	GeometryRecognizeDomainHandler m_geomHandler;
	std::vector<std::shared_ptr<IAiInferenceProvider>> m_inferenceProviders;
	QString m_lastLoadedModel;
};
