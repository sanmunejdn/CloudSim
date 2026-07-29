/// @file AiAssistantHostImpl.cpp
/// @brief AiAssistantHostImpl 实现

#include "Ai/AiAssistantHostImpl.h"

#include "Ai/AiActionPlanExecutor.h"
#include "Ai/AiAgentRuntime.h"
#include "Ai/AiApiCatalogEmbedded.h"
#include "Ai/AiCatalogKeywordMatcher.h"
#include "Ai/AiConfigLoader.h"
#include "Ai/AiTrajectoryFeatureCatalog.h"
#include "Ai/MeshComposeDomainHandler.h"
#include "AiIntentParser.h"
#include "AiLlmClient.h"
#include "AiProgressSink.h"
#include "AiTrajectoryFeatureTypes.h"
#include "CloudSimAiVersion.h"
#include "PluginHostContext.h"

#include <json.hpp>

namespace
{
AiLlmConfig toLlmConfig(const AiRemoteLlmConfig& r)
{
	AiLlmConfig c;
	c.enabled = r.enabled;
	c.baseUrl = r.baseUrl;
	c.apiKey = r.apiKey;
	c.apiKeyEnv = r.apiKeyEnv;
	c.model = r.model;
	c.timeoutMs = r.timeoutMs;
	c.temperature = r.temperature;
	return c;
}

AiLlmConfig domainToLlmConfig(const AiDomainModelConfig& d)
{
	AiLlmConfig c;
	c.enabled = true;
	c.baseUrl = d.baseUrl;
	c.model = d.model;
	c.timeoutMs = 120000;
	c.temperature = 0.1;
	return c;
}

AiInferenceProgressFn wrapProgressForUi(PluginHostContext* host, const AiInferenceProgressFn& progress)
{
	if (!progress || !host)
		return progress;
	return [host, progress](double fraction, const QString& message)
	{
		host->invokeOnUiThread(
			[progress, fraction, message]()
			{
				if (progress)
					progress(fraction, message);
			});
	};
}

bool isConversationalQuery(const QString& text)
{
	const QString t = text.trimmed();
	if (t.isEmpty())
		return false;
	static const QStringList keys = {
		QStringLiteral("什么模型"), QStringLiteral("哪个模型"), QStringLiteral("你是谁"),	QStringLiteral("你是什么"),
		QStringLiteral("你好"),		QStringLiteral("介绍一下"), QStringLiteral("能做什么"),
	};
	for (const QString& k : keys)
	{
		if (t.contains(k))
			return true;
	}
	return false;
}
} // namespace

AiAssistantHostImpl::AiAssistantHostImpl(PluginHostContext* pluginHost)
	: m_pluginHost(pluginHost), m_router(&m_registry)
{
	registerBuiltinDomains();
	m_agentRuntime = std::make_unique<AiAgentRuntime>(pluginHost, this);
}

AiAssistantHostImpl::~AiAssistantHostImpl() = default;

unsigned int AiAssistantHostImpl::aiSdkVersion() const
{
	return cloudsimAiSdkVersion();
}

std::optional<AiConfigDto> AiAssistantHostImpl::loadConfig() const
{
	return loadAiConfigDto();
}

bool AiAssistantHostImpl::saveConfig(const AiConfigDto& config, QString* errorMessage) const
{
	return saveAiConfigDto(config, QString(), errorMessage);
}

QByteArray AiAssistantHostImpl::apiCatalogJson() const
{
	return aiEmbeddedApiCatalogJson();
}

IAiDomainRegistry* AiAssistantHostImpl::domainRegistry()
{
	return &m_registry;
}

const IAiDomainRegistry* AiAssistantHostImpl::domainRegistry() const
{
	return &m_registry;
}

void AiAssistantHostImpl::registerInferenceProvider(std::shared_ptr<IAiInferenceProvider> provider)
{
	if (provider)
		m_inferenceProviders.push_back(std::move(provider));
}

void AiAssistantHostImpl::registerBuiltinDomains()
{
	const QByteArray catalog = apiCatalogJson();
	{
		AiDomainDescriptor d;
		d.domainId = AiDomainIds::meshCreate();
		d.displayName = QStringLiteral("Create mesh");
		d.outputKind = AiDomainOutputKind::ActionPlan;
		d.supportsMultimodal = false;
		d.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local"), QStringLiteral("remote")};
		m_registry.registerDomain(d, &m_meshHandler);
	}
	{
		AiDomainDescriptor d;
		d.domainId = AiDomainIds::meshCompose();
		d.displayName = QStringLiteral("Compose mesh (boolean)");
		d.outputKind = AiDomainOutputKind::ActionPlan;
		d.supportsMultimodal = false;
		d.parserPriority = QStringList{QStringLiteral("local"), QStringLiteral("remote")};
		m_registry.registerDomain(d, &m_composeHandler);
	}
	{
		AiDomainDescriptor d;
		d.domainId = AiDomainIds::featureCompose();
		d.displayName = QStringLiteral("Compose parametric features");
		d.outputKind = AiDomainOutputKind::ActionPlan;
		d.supportsMultimodal = false;
		d.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local"), QStringLiteral("remote")};
		m_registry.registerDomain(d, &m_featureComposeHandler);
	}
	{
		AiDomainDescriptor d;
		d.domainId = AiDomainIds::geometryRecognize();
		d.displayName = QStringLiteral("Geometry recognize");
		d.outputKind = AiDomainOutputKind::StructuredJson;
		d.supportsMultimodal = true;
		d.parserPriority = QStringList{QStringLiteral("local")};
		d.unloadOtherModelsBeforeInfer = true;
		m_registry.registerDomain(d, &m_geomHandler);
	}
	{
		AiDomainDescriptor d;
		d.domainId = AiDomainIds::trajectoryFeature();
		d.displayName = QStringLiteral("Trajectory feature");
		d.outputKind = AiDomainOutputKind::StructuredJson;
		d.supportsMultimodal = false;
		d.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local"), QStringLiteral("remote")};
		m_registry.registerDomain(d, &m_trajFeatureHandler);
	}

	m_docImportHandler.emplace(AiDomainIds::documentImport(), catalog);
	m_pcOpsHandler.emplace(AiDomainIds::pointCloudOps(), catalog);
	m_geomOpsHandler.emplace(AiDomainIds::geometryOps(), catalog);
	m_featureHandler.emplace(AiDomainIds::featureBuild(), catalog);
	m_labelHandler.emplace(AiDomainIds::labelingAnnot(), catalog);
	m_sceneOpsHandler.emplace(AiDomainIds::sceneOps(), catalog);
	m_processFlowHandler.emplace(AiDomainIds::processFlow(), catalog);

	auto regCatalog = [this](const QString& id, const QString& name, CatalogActionPlanDomainHandler* h)
	{
		AiDomainDescriptor d;
		d.domainId = id;
		d.displayName = name;
		d.outputKind = AiDomainOutputKind::ActionPlan;
		d.supportsMultimodal = false;
		d.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local")};
		m_registry.registerDomain(d, h);
	};
	regCatalog(AiDomainIds::documentImport(), QStringLiteral("Document import"), &*m_docImportHandler);
	regCatalog(AiDomainIds::pointCloudOps(), QStringLiteral("Point cloud ops"), &*m_pcOpsHandler);
	regCatalog(AiDomainIds::geometryOps(), QStringLiteral("Geometry ops"), &*m_geomOpsHandler);
	regCatalog(AiDomainIds::featureBuild(), QStringLiteral("Feature build"), &*m_featureHandler);
	regCatalog(AiDomainIds::labelingAnnot(), QStringLiteral("Labeling"), &*m_labelHandler);
	regCatalog(AiDomainIds::sceneOps(), QStringLiteral("Scene ops"), &*m_sceneOpsHandler);
	regCatalog(AiDomainIds::processFlow(), QStringLiteral("Process flow"), &*m_processFlowHandler);
}

QString AiAssistantHostImpl::resolveDomainId(const QString& requestedDomainId, const QString& userText) const
{
	return m_router.resolve(requestedDomainId, userText);
}

AiParseResult AiAssistantHostImpl::parseUserTextWithRules(const QString& domainId, const QString& text) const
{
	AiParseResult r;
	QString d = domainId;
	if (d.isEmpty() || d == AiDomainIds::autoDomain())
		d = resolveDomainId(AiDomainIds::autoDomain(), text);
	r.domainId = d;

	const bool catalogDomain = d == AiDomainIds::documentImport() || d == AiDomainIds::pointCloudOps() ||
							   d == AiDomainIds::geometryOps() || d == AiDomainIds::featureBuild() ||
							   d == AiDomainIds::labelingAnnot() || d == AiDomainIds::sceneOps() ||
							   d == AiDomainIds::processFlow();
	if (catalogDomain)
	{
		const auto m = AiCatalogKeywordMatcher::tryMatch(apiCatalogJson(), text, d);
		r.ok = m.ok;
		r.outputKind = AiDomainOutputKind::ActionPlan;
		r.parserVia = QStringLiteral("Rules");
		if (m.ok)
		{
			r.outputJsonUtf8 = m.planJsonUtf8;
			r.hintMessage = m.hintMessage;
			if (!m.domainId.isEmpty())
				r.domainId = m.domainId;
		}
		else
		{
			r.errorMessage = m.errorMessage;
			r.hintMessage = m.hintMessage;
		}
		return r;
	}

	if (d == AiDomainIds::meshCreate())
	{
		const AiIntentParser::ParseResult pr = AiIntentParser::tryParseUserText(text);
		r.ok = pr.ok;
		r.outputKind = AiDomainOutputKind::ActionPlan;
		if (pr.ok)
		{
			r.outputJsonUtf8 = QByteArray::fromStdString(pr.command.dump());
			r.hintMessage = pr.hintMessage;
			r.parserVia = QStringLiteral("Rules");
		}
		else
		{
			r.errorMessage = pr.errorMessage;
			r.hintMessage = pr.hintMessage;
			r.parserVia = QStringLiteral("Rules");
		}
		return r;
	}
	if (d == AiDomainIds::meshCompose())
	{
		const AiIntentParser::ParseResult pr = AiIntentParser::tryParseComposeUserText(text);
		r.ok = pr.ok;
		r.outputKind = AiDomainOutputKind::ActionPlan;
		if (pr.ok)
		{
			r.outputJsonUtf8 = QByteArray::fromStdString(pr.command.dump());
			r.hintMessage = pr.hintMessage;
			r.parserVia = QStringLiteral("Rules");
		}
		else
		{
			r.errorMessage = pr.errorMessage;
			r.hintMessage = pr.hintMessage;
			r.parserVia = QStringLiteral("Rules");
		}
		return r;
	}
	if (d == AiDomainIds::featureCompose())
	{
		const AiIntentParser::ParseResult pr = AiIntentParser::tryParseFeatureComposeUserText(text);
		r.ok = pr.ok;
		r.outputKind = AiDomainOutputKind::ActionPlan;
		if (pr.ok)
		{
			r.outputJsonUtf8 = QByteArray::fromStdString(pr.command.dump());
			r.hintMessage = pr.hintMessage;
			r.parserVia = QStringLiteral("Rules");
		}
		else
		{
			r.errorMessage = pr.errorMessage;
			r.hintMessage = pr.hintMessage;
			r.parserVia = QStringLiteral("Rules");
		}
		return r;
	}
	if (d == AiDomainIds::trajectoryFeature())
	{
		r.ok = false;
		r.errorMessage =
			QStringLiteral("trajectory.feature rules require catalog context; use parseTrajectoryFeatureRequest.");
		r.parserVia = QStringLiteral("Rules");
		return r;
	}
	r.errorMessage = QStringLiteral("No rule parser for domain %1.").arg(r.domainId);
	return r;
}

AiParseResult AiAssistantHostImpl::parseTrajectoryFeatureRequest(const AiInferenceRequest& request) const
{
	AiParseResult r;
	r.domainId = AiDomainIds::trajectoryFeature();
	r.outputKind = AiDomainOutputKind::StructuredJson;
	const AiFeatureAxis axis = AiTrajectoryFeatureCatalog::inferFeatureAxisFromText(request.userText);
	return AiTrajectoryFeatureCatalog::tryParseTrajectoryFeatureRules(
		request.userText, axis, request.catalogSliceUtf8, request.workpieceBackendId, request.workpieceStepPathUtf8);
}

const AiDomainModelConfig* AiAssistantHostImpl::findDomainConfig(const AiConfigDto& cfg, const QString& domainId) const
{
	for (const auto& d : cfg.domains)
	{
		if (d.id == domainId && d.enabled)
			return &d;
	}
	return nullptr;
}

AiParseResult AiAssistantHostImpl::parseWithLocalLlm(const QString& domainId, const QString& text,
													 const AiDomainModelConfig& dm, const QByteArray& imagePng,
													 const AiInferenceProgressFn& progress,
													 const QByteArray& catalogSliceUtf8) const
{
	AiParseResult r;
	r.domainId = domainId;
	const AiLlmConfig llm = domainToLlmConfig(dm);
	const AiProgressSink sink = [&progress](double f, const QString& m)
	{
		if (progress)
			progress(f, m);
	};
	const AiLlmClient::LlmParseResult lr =
		AiLlmClient::parseUserTextWithLlm(text, llm, sink, imagePng, domainId, catalogSliceUtf8);
	r.ok = lr.ok;
	if (domainId == AiDomainIds::geometryRecognize() || domainId == AiDomainIds::trajectoryFeature())
		r.outputKind = AiDomainOutputKind::StructuredJson;
	else
		r.outputKind = AiDomainOutputKind::ActionPlan;
	if (lr.ok)
	{
		r.outputJsonUtf8 = QByteArray::fromStdString(lr.command.dump());
		r.parserVia = QStringLiteral("Local %1").arg(dm.model);
	}
	else
	{
		r.errorMessage = lr.errorMessage;
		r.parserVia = QStringLiteral("Local %1").arg(dm.model);
	}
	return r;
}

AiParseResult AiAssistantHostImpl::parseWithRemoteLlm(const QString& domainId, const QString& text,
													  const AiRemoteLlmConfig& remote,
													  const AiInferenceProgressFn& progress,
													  const QByteArray& catalogSliceUtf8) const
{
	AiParseResult r;
	r.domainId = domainId;
	if (!remote.enabled)
	{
		r.errorMessage = QStringLiteral("Remote LLM disabled.");
		return r;
	}
	AiLlmConfig llm = toLlmConfig(remote);
	if (!llm.hasApiKey())
	{
		r.errorMessage = QStringLiteral("Remote API key not configured.");
		return r;
	}
	const AiProgressSink sink = [&progress](double f, const QString& m)
	{
		if (progress)
			progress(f, m);
	};
	const AiLlmClient::LlmParseResult lr =
		AiLlmClient::parseUserTextWithLlm(text, llm, sink, QByteArray(), domainId, catalogSliceUtf8);
	r.ok = lr.ok;
	if (domainId == AiDomainIds::geometryRecognize() || domainId == AiDomainIds::trajectoryFeature())
		r.outputKind = AiDomainOutputKind::StructuredJson;
	else
		r.outputKind = AiDomainOutputKind::ActionPlan;
	if (lr.ok)
	{
		r.outputJsonUtf8 = QByteArray::fromStdString(lr.command.dump());
		r.parserVia = QStringLiteral("Remote %1").arg(remote.model);
	}
	else
	{
		r.errorMessage = lr.errorMessage;
		r.parserVia = QStringLiteral("Remote %1").arg(remote.model);
	}
	return r;
}

void AiAssistantHostImpl::parseUserTextAsync(const AiInferenceRequest& request, const AiConfigDto& config,
											 const AiInferenceProgressFn& progress,
											 std::function<void(AiParseResult)> onFinished)
{
	if (!m_pluginHost || !onFinished)
		return;

	const QString domainId = resolveDomainId(request.domainId, request.userText);
	if (domainId == AiDomainIds::geometryRecognize() && request.imagePng.isEmpty())
	{
		AiParseResult r;
		r.domainId = domainId;
		r.ok = false;
		r.errorMessage = QStringLiteral("几何识别需要当前 3D 视口截图。请先打开含视口的文档并重试。");
		r.hintMessage = QStringLiteral("在 AI 面板选择「几何识别」后发送识别指令。");
		onFinished(r);
		return;
	}

	QStringList chain = config.parserPriorityDefault;
	if (const AiDomainDescriptor* descUi = m_registry.descriptor(domainId))
	{
		if (!descUi->parserPriority.isEmpty())
			chain = descUi->parserPriority;
	}

	const auto result = std::make_shared<AiParseResult>();
	const AiInferenceRequest reqCopy = request;
	const QString userText = request.userText;
	const QByteArray imagePng = request.imagePng;
	const AiConfigDto cfgCopy = config;
	const AiInferenceProgressFn uiProgress = wrapProgressForUi(m_pluginHost, progress);

	m_pluginHost->enqueueJob(
		QStringLiteral("AI: parse"),
		[this, reqCopy, userText, imagePng, cfgCopy, domainId, chain, uiProgress, result](const PluginJobProgressFn&)
		{
			const AiDomainDescriptor* desc = m_registry.descriptor(domainId);
			const AiDomainModelConfig* dm = findDomainConfig(cfgCopy, domainId);

			result->domainId = domainId;
			if (desc)
				result->outputKind = desc->outputKind;
			else
				result->outputKind = AiDomainOutputKind::ActionPlan;

			for (const QString& step : chain)
			{
				if (step == QStringLiteral("rules"))
				{
					const AiParseResult rr = domainId == AiDomainIds::trajectoryFeature()
												 ? parseTrajectoryFeatureRequest(reqCopy)
												 : parseUserTextWithRules(domainId, userText);
					if (rr.ok)
					{
						*result = rr;
						return;
					}
					*result = rr;
				}
				else if (step == QStringLiteral("local") && dm)
				{
					if (isConversationalQuery(userText))
					{
						result->ok = false;
						result->errorMessage = QStringLiteral("我是 CloudSim AI 助手（规则解析 + 本地模型 %1）。\n"
															  "创建几何请使用「生成/创建」+ 基本体类型 + 尺寸（mm）。")
												   .arg(dm->model);
						result->hintMessage = QStringLiteral("示例：生成长方体（可省略尺寸，将用默认 100×100×100 mm）");
						result->parserVia = QStringLiteral("Local");
						return;
					}
					if (dm->unloadOtherModelsBeforeInfer && !m_lastLoadedModel.isEmpty() &&
						m_lastLoadedModel != dm->model)
					{
						m_lastLoadedModel.clear();
					}
					m_lastLoadedModel = dm->model;
					const AiParseResult lr =
						parseWithLocalLlm(domainId, userText, *dm, imagePng, uiProgress, reqCopy.catalogSliceUtf8);
					if (lr.ok)
					{
						*result = lr;
						return;
					}
					if (result->errorMessage.isEmpty())
						*result = lr;
				}
				else if (step == QStringLiteral("remote"))
				{
					const AiParseResult rr =
						parseWithRemoteLlm(domainId, userText, cfgCopy.remoteLlm, uiProgress, reqCopy.catalogSliceUtf8);
					if (rr.ok)
					{
						*result = rr;
						return;
					}
					if (result->errorMessage.isEmpty())
						*result = rr;
				}
			}
			if (!result->ok && result->errorMessage.isEmpty())
				result->errorMessage = QStringLiteral("All parsers failed for domain %1.").arg(domainId);
		},
		[result, onFinished](bool threw, const QString& throwMsg)
		{
			if (threw)
			{
				AiParseResult r;
				r.errorMessage = throwMsg.isEmpty() ? QStringLiteral("AI parse job failed.") : throwMsg;
				onFinished(r);
				return;
			}
			onFinished(*result);
		});
}

bool AiAssistantHostImpl::executeActionPlan(const QByteArray& actionPlanJsonUtf8, QString* outSummary,
											QString* outError)
{
	if (!m_pluginHost)
	{
		if (outError)
			*outError = QStringLiteral("Plugin host not available.");
		return false;
	}
	return AiActionPlanExecutor::execute(*m_pluginHost, actionPlanJsonUtf8, outSummary, outError);
}

bool AiAssistantHostImpl::executeDomainOutput(const QString& domainId, const QByteArray& outputJsonUtf8,
											  QString* outSummary, QString* outError)
{
	IAiDomainHandler* h = m_registry.handler(domainId);
	if (!h || !m_pluginHost)
	{
		if (outError)
			*outError = QStringLiteral("Unknown domain or host.");
		return false;
	}
	QString err;
	if (!h->validateOutput(outputJsonUtf8, &err))
	{
		if (outError)
			*outError = err;
		return false;
	}
	return h->execute(outputJsonUtf8, m_pluginHost, this, outSummary, outError);
}

void AiAssistantHostImpl::runAgentTurnAsync(const AiInferenceRequest& request, const AiConfigDto& config,
											const AiInferenceProgressFn& progress, const AiAgentEventFn& onEvent)
{
	if (!m_agentRuntime)
	{
		if (onEvent)
		{
			AiAgentEvent ev;
			ev.kind = AiAgentEventKind::Error;
			ev.message = QStringLiteral("Agent runtime unavailable.");
			ev.isError = true;
			onEvent(ev);
		}
		return;
	}
	const AiInferenceProgressFn uiProgress = wrapProgressForUi(m_pluginHost, progress);
	const AiAgentEventFn uiEvent = [this, onEvent](const AiAgentEvent& ev)
	{
		if (!m_pluginHost)
		{
			if (onEvent)
				onEvent(ev);
			return;
		}
		m_pluginHost->invokeOnUiThread([onEvent, ev]()
									   {
										   if (onEvent)
											   onEvent(ev);
									   });
	};
	m_agentRuntime->runTurnAsync(request, config, uiProgress, uiEvent);
}

void AiAssistantHostImpl::submitAgentConfirm(const QString& pendingId, const QByteArray& argsJsonUtf8)
{
	if (m_agentRuntime)
		m_agentRuntime->submitConfirm(pendingId, argsJsonUtf8);
}

void AiAssistantHostImpl::cancelAgentConfirm(const QString& pendingId)
{
	if (m_agentRuntime)
		m_agentRuntime->cancelConfirm(pendingId);
}

void AiAssistantHostImpl::cancelAgentTurn()
{
	if (m_agentRuntime)
		m_agentRuntime->cancelTurn();
}

void AiAssistantHostImpl::beginDomainConfirmAsync(const AiDomainConfirmRequest& request, const AiAgentEventFn& onEvent)
{
	if (!m_agentRuntime)
	{
		if (onEvent)
		{
			AiAgentEvent ev;
			ev.kind = AiAgentEventKind::Error;
			ev.message = QStringLiteral("Agent runtime unavailable.");
			ev.isError = true;
			onEvent(ev);
		}
		return;
	}
	const AiAgentEventFn uiEvent = [this, onEvent](const AiAgentEvent& ev)
	{
		if (!m_pluginHost)
		{
			if (onEvent)
				onEvent(ev);
			return;
		}
		m_pluginHost->invokeOnUiThread([onEvent, ev]()
									   {
										   if (onEvent)
											   onEvent(ev);
									   });
	};
	m_agentRuntime->beginDomainConfirm(request, uiEvent);
}

void AiAssistantHostImpl::secondaryAgentConfirm(const QString& pendingId)
{
	if (m_agentRuntime)
		m_agentRuntime->secondaryConfirm(pendingId);
}
