#ifndef CLOUDSIMPLUGINHOST_AIASSISTANTHOSTIMPL_H
#define CLOUDSIMPLUGINHOST_AIASSISTANTHOSTIMPL_H

/// @file AiAssistantHostImpl.h
/// @brief AiAssistantHostImpl 接口

#include "Ai/AiDomainRegistryImpl.h"
#include "Ai/AiDomainRouter.h"
#include "Ai/CatalogActionPlanDomainHandler.h"
#include "Ai/GeometryRecognizeDomainHandler.h"
#include "Ai/MeshComposeDomainHandler.h"
#include "Ai/MeshCreateDomainHandler.h"
#include "Ai/TrajectoryFeatureDomainHandler.h"
#include "IAiAssistantHost.h"

#include <memory>
#include <optional>

class AiAgentRuntime;
class PluginHostContext;

class AiAssistantHostImpl : public IAiAssistantHost
{
public:
	explicit AiAssistantHostImpl(PluginHostContext* pluginHost);
	~AiAssistantHostImpl() override;

	unsigned int aiSdkVersion() const override;
	std::optional<AiConfigDto> loadConfig() const override;
	bool saveConfig(const AiConfigDto& config, QString* errorMessage = nullptr) const override;

	QByteArray apiCatalogJson() const override;
	IAiDomainRegistry* domainRegistry() override;
	const IAiDomainRegistry* domainRegistry() const override;

	void registerInferenceProvider(std::shared_ptr<IAiInferenceProvider> provider) override;

	AiParseResult parseUserTextWithRules(const QString& domainId, const QString& text) const override;

	AiParseResult parseTrajectoryFeatureRequest(const AiInferenceRequest& request) const override;

	void parseUserTextAsync(const AiInferenceRequest& request, const AiConfigDto& config,
							const AiInferenceProgressFn& progress,
							std::function<void(AiParseResult)> onFinished) override;

	bool executeActionPlan(const QByteArray& actionPlanJsonUtf8, QString* outSummary, QString* outError) override;

	bool executeDomainOutput(const QString& domainId, const QByteArray& outputJsonUtf8, QString* outSummary,
							 QString* outError) override;

	QString resolveDomainId(const QString& requestedDomainId, const QString& userText) const override;

	void runAgentTurnAsync(const AiInferenceRequest& request, const AiConfigDto& config,
						   const AiInferenceProgressFn& progress, const AiAgentEventFn& onEvent) override;
	void submitAgentConfirm(const QString& pendingId, const QByteArray& argsJsonUtf8) override;
	void cancelAgentConfirm(const QString& pendingId) override;
	void cancelAgentTurn() override;
	void beginDomainConfirmAsync(const AiDomainConfirmRequest& request, const AiAgentEventFn& onEvent) override;
	void secondaryAgentConfirm(const QString& pendingId) override;

private:
	void registerBuiltinDomains();
	const AiDomainModelConfig* findDomainConfig(const AiConfigDto& cfg, const QString& domainId) const;
	AiParseResult parseWithLocalLlm(const QString& domainId, const QString& text, const AiDomainModelConfig& dm,
									const QByteArray& imagePng, const AiInferenceProgressFn& progress,
									const QByteArray& catalogSliceUtf8 = QByteArray()) const;
	AiParseResult parseWithRemoteLlm(const QString& domainId, const QString& text, const AiRemoteLlmConfig& remote,
									 const AiInferenceProgressFn& progress,
									 const QByteArray& catalogSliceUtf8 = QByteArray()) const;

	PluginHostContext* m_pluginHost = nullptr;
	AiDomainRegistryImpl m_registry;
	AiDomainRouter m_router;
	MeshCreateDomainHandler m_meshHandler;
	MeshComposeDomainHandler m_composeHandler;
	GeometryRecognizeDomainHandler m_geomHandler;
	TrajectoryFeatureDomainHandler m_trajFeatureHandler;
	std::optional<CatalogActionPlanDomainHandler> m_docImportHandler;
	std::optional<CatalogActionPlanDomainHandler> m_pcOpsHandler;
	std::optional<CatalogActionPlanDomainHandler> m_geomOpsHandler;
	std::optional<CatalogActionPlanDomainHandler> m_featureHandler;
	std::optional<CatalogActionPlanDomainHandler> m_labelHandler;
	std::optional<CatalogActionPlanDomainHandler> m_sceneOpsHandler;
	std::optional<CatalogActionPlanDomainHandler> m_processFlowHandler;
	std::vector<std::shared_ptr<IAiInferenceProvider>> m_inferenceProviders;
	QString m_lastLoadedModel;
	std::unique_ptr<AiAgentRuntime> m_agentRuntime;
};

#endif
