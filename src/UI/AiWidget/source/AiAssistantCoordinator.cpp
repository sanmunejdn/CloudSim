/// @file AiAssistantCoordinator.cpp
/// @brief AiAssistantCoordinator 实现

#include "AiAssistantCoordinator.h"

#include "AiAgentTypes.h"
#include "AiAssistantDockWidget.h"
#include "AiConfigDefaults.h"
#include "AiConfigDto.h"
#include "AiDomainTypes.h"
#include "AiInferenceTypes.h"
#include "AiTrajectoryFeatureTypes.h"
#include "IAiAssistantHost.h"
#include "IPluginHostContext.h"

#include <QTimer>
#include <set>
#include <vector>

#include <json.hpp>

namespace
{
QString prefixWithParser(const QString& parserVia, const QString& text)
{
	if (parserVia.isEmpty())
		return text;
	return QStringLiteral("[%1] %2").arg(parserVia, text);
}

const AiDomainModelConfig* findDomainConfig(const AiConfigDto& cfg, const QString& domainId)
{
	for (const AiDomainModelConfig& d : cfg.domains)
	{
		if (d.id == domainId)
			return &d;
	}
	return nullptr;
}

QString formatFeatureCandidateList(const QByteArray& catalogSliceUtf8, bool chinese)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		QString body;
		if (!j.contains("candidates") || !j["candidates"].is_array())
		{
			return body;
		}
		for (const auto& c : j["candidates"])
		{
			const int idx = c.value("displayIndex", 0);
			const QString id = QString::fromStdString(c.value("candidateId", std::string()));
			const QString summary = QString::fromStdString(c.value("summary", std::string()));
			const QString kind = QString::fromStdString(c.value("suggestedKind", std::string()));
			if (chinese)
			{
				body += QStringLiteral("%1. [%2] %3 — %4\n").arg(idx).arg(kind, id, summary);
			}
			else
			{
				body += QStringLiteral("%1. [%2] %3 — %4\n").arg(idx).arg(kind, id, summary);
			}
		}
		return body;
	}
	catch (...)
	{
		return QString();
	}
}

bool planIsAmbiguous(const QByteArray& jsonUtf8, QString* clarifyOut)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
		const std::string axis = j.value("featureAxis", std::string());
		if (axis == "ambiguous")
		{
			if (clarifyOut)
			{
				*clarifyOut = QString::fromStdString(j.value("clarifyMessage", std::string()));
			}
			return true;
		}
	}
	catch (...)
	{
	}
	return false;
}

AiFeatureAxis inferFeatureAxisLocal(const QString& userText)
{
	const QString t = userText.trimmed();
	auto contains = [&](const QStringList& keys)
	{
		for (const QString& k : keys)
		{
			if (t.contains(k))
				return true;
		}
		return false;
	};
	if (contains({QStringLiteral("面特征"), QStringLiteral("大平面"), QStringLiteral("栅格"), QStringLiteral("打磨"),
				  QStringLiteral("grind"), QStringLiteral("UV")}) ||
		(contains({QStringLiteral("面"), QStringLiteral("surface"), QStringLiteral("face")}) &&
		 !contains({QStringLiteral("面特征")})))
	{
		return AiFeatureAxis::Surface;
	}
	if (contains({QStringLiteral("线特征"), QStringLiteral("边"), QStringLiteral("焊缝"), QStringLiteral("交线"),
				  QStringLiteral("轮廓"), QStringLiteral("涂胶"), QStringLiteral("weld"), QStringLiteral("glue"),
				  QStringLiteral("seam")}))
	{
		return AiFeatureAxis::Line;
	}
	return AiFeatureAxis::Ambiguous;
}

bool parseDisplayIndexSelectionLocal(const QString& userText, const QByteArray& catalogSliceUtf8,
									 std::vector<std::string>& outCandidateIds, QString* err)
{
	outCandidateIds.clear();
	std::set<int> indices;
	const QString t = userText;
	for (int i = 0; i < t.size(); ++i)
	{
		if (t[i].isDigit())
		{
			int val = 0;
			while (i < t.size() && t[i].isDigit())
			{
				val = val * 10 + t[i].digitValue();
				++i;
			}
			if (val > 0)
				indices.insert(val);
			--i;
		}
	}
	if (indices.empty())
	{
		if (err)
			*err = QStringLiteral("未解析到编号，请使用如「选 1 和 3」。");
		return false;
	}
	try
	{
		const nlohmann::json j = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		for (const auto& item : j["candidates"])
		{
			const int displayIndex = item.value("displayIndex", 0);
			if (indices.count(displayIndex) > 0)
				outCandidateIds.push_back(item.value("candidateId", std::string()));
		}
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("Catalog slice parse failed.");
		return false;
	}
	if (outCandidateIds.empty())
	{
		if (err)
			*err = QStringLiteral("编号超出当前候选范围。");
		return false;
	}
	return true;
}

QByteArray filterCatalogSliceByCandidateIds(const QByteArray& catalogSliceUtf8,
											const std::vector<std::string>& candidateIds)
{
	if (candidateIds.empty())
	{
		return catalogSliceUtf8;
	}
	try
	{
		const nlohmann::json src = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		nlohmann::json dst = src;
		dst["candidates"] = nlohmann::json::array();
		if (!src.contains("candidates") || !src["candidates"].is_array())
		{
			return catalogSliceUtf8;
		}
		std::set<std::string> idSet(candidateIds.begin(), candidateIds.end());
		for (const auto& c : src["candidates"])
		{
			const std::string id = c.value("candidateId", std::string());
			if (idSet.count(id) > 0)
			{
				dst["candidates"].push_back(c);
			}
		}
		return QByteArray::fromStdString(dst.dump());
	}
	catch (...)
	{
		return catalogSliceUtf8;
	}
}

QByteArray buildMinimalFeaturePlanFromSelection(const std::vector<std::string>& candidateIds,
												const QByteArray& catalogFullUtf8, const QString& backendId,
												const QString& stepPath, const QString& pipelineTemplate)
{
	try
	{
		const nlohmann::json full = nlohmann::json::parse(catalogFullUtf8.constData(), nullptr, true);
		nlohmann::json plan;
		plan["version"] = 1;
		plan["selectedCandidateIds"] = candidateIds;
		plan["suggestedPipelineTemplate"] = pipelineTemplate.toStdString();
		nlohmann::json feats = nlohmann::json::array();
		for (const std::string& id : candidateIds)
		{
			for (const auto& c : full["candidates"])
			{
				if (c.value("candidateId", std::string()) != id)
					continue;
				nlohmann::json spec;
				spec["schemaVersion"] = 1;
				spec["featureId"] = id;
				spec["kind"] = c.value("suggestedKind", std::string("EdgeChain"));
				spec["workpiece"] = {
					{"backendIdUtf8", backendId.toStdString()},
					{"stepPathUtf8", stepPath.toStdString()},
				};
				spec["refs"] = c.value("refs", nlohmann::json::object());
				spec["discretize"] = {
					{"stepMm", 5.0}, {"linearDeflectionMm", 0.01}, {"outputTangent", true}, {"outputNormal", true}};
				feats.push_back(spec);
				break;
			}
		}
		plan["features"] = feats;
		return QByteArray::fromStdString(plan.dump());
	}
	catch (...)
	{
		return QByteArray();
	}
}

bool catalogSliceHasCandidates(const QByteArray& catalogSliceUtf8)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		return j.contains("candidates") && j["candidates"].is_array() && !j["candidates"].empty();
	}
	catch (...)
	{
		return false;
	}
}

bool isCatalogEmptyTrajectoryMessage(const QString& msg)
{
	const QString t = msg.trimmed().toLower();
	if (t.isEmpty())
	{
		return false;
	}
	return t.contains(QStringLiteral("no features in catalog")) ||
		   t.contains(QStringLiteral("catalog slice invalid")) ||
		   (t.contains(QStringLiteral("catalog")) && t.contains(QStringLiteral("select"))) ||
		   t.contains(QStringLiteral("未找到匹配的特征候选"));
}

} // namespace

AiAssistantCoordinator::AiAssistantCoordinator(AiAssistantDockWidget* dock, QObject* parent)
	: QObject(parent), m_dock(dock)
{
}

void AiAssistantCoordinator::setAiHost(IAiAssistantHost* host)
{
	m_aiHost = host;
}

void AiAssistantCoordinator::setPluginHost(IPluginHostContext* host)
{
	m_pluginHost = host;
}

void AiAssistantCoordinator::resetFeatureSession()
{
	m_featureSessionState = FeatureSessionState::Idle;
	m_pendingFeaturePlanJson.clear();
	m_pendingCatalogSliceUtf8.clear();
	m_pendingCatalogFullUtf8.clear();
	m_pendingFeatureParserVia.clear();
	m_pendingWorkpieceBackendId.clear();
	m_pendingWorkpieceStepPath.clear();
	m_pendingFeatureAxis = AiFeatureAxis::Ambiguous;
	m_pendingPipelineTemplate.clear();
	if (m_pluginHost)
	{
		m_pluginHost->clearAiFeatureCandidatePreview();
	}
	if (m_dock)
	{
		m_dock->hideTrajectoryFeatureConfirmButtons();
	}
}

bool AiAssistantCoordinator::prepareTrajectoryFeatureRequest(const QString& userText, AiInferenceRequest& req,
															 QString* err)
{
	if (!m_pluginHost)
	{
		if (err)
		{
			*err = QStringLiteral("宿主未就绪");
		}
		return false;
	}
	QString backendId;
	QString stepPath;
	if (!m_pluginHost->resolveTrajectoryWorkpiece(backendId, stepPath, err))
	{
		return false;
	}
	req.workpieceBackendId = backendId;
	req.workpieceStepPathUtf8 = stepPath;
	if (!m_pluginHost->buildTrajectoryFeatureCatalogSlice(backendId, stepPath, userText, req.catalogFullUtf8,
														  req.catalogSliceUtf8, err))
	{
		return false;
	}
	return true;
}

bool AiAssistantCoordinator::tryHandleFeatureFollowUp(const QString& text)
{
	if (m_featureSessionState == FeatureSessionState::Idle || !m_pluginHost)
	{
		return false;
	}

	if (m_featureSessionState == FeatureSessionState::AwaitingAxisClarify)
	{
		AiFeatureAxis axis = AiFeatureAxis::Ambiguous;
		if (text.contains(QStringLiteral("线")) || text.contains(QStringLiteral("边")) ||
			text.contains(QStringLiteral("line"), Qt::CaseInsensitive))
		{
			axis = AiFeatureAxis::Line;
		}
		else if (text.contains(QStringLiteral("面")) || text.contains(QStringLiteral("surface"), Qt::CaseInsensitive))
		{
			axis = AiFeatureAxis::Surface;
		}
		else
		{
			return false;
		}
		m_pendingFeatureAxis = axis;
		if (!m_pluginHost->buildTrajectoryFeatureCatalogSlice(m_pendingWorkpieceBackendId, m_pendingWorkpieceStepPath,
															  text, m_pendingCatalogFullUtf8, m_pendingCatalogSliceUtf8,
															  nullptr))
		{
			return false;
		}
		AiInferenceRequest req;
		req.domainId = AiDomainIds::trajectoryFeature();
		req.userText = text;
		req.workpieceBackendId = m_pendingWorkpieceBackendId;
		req.workpieceStepPathUtf8 = m_pendingWorkpieceStepPath;
		req.catalogFullUtf8 = m_pendingCatalogFullUtf8;
		req.catalogSliceUtf8 = m_pendingCatalogSliceUtf8;
		if (!m_aiHost)
		{
			return false;
		}
		const AiParseResult rules = m_aiHost->parseTrajectoryFeatureRequest(req);
		if (!rules.ok)
		{
			if (m_dock)
			{
				m_dock->appendAssistantMessage(prefixWithParser(rules.parserVia, rules.errorMessage));
			}
			return true;
		}
		handleTrajectoryParseResult(rules);
		return true;
	}

	if (m_featureSessionState == FeatureSessionState::PreviewCandidates ||
		m_featureSessionState == FeatureSessionState::AwaitingSelection)
	{
		if (text.contains(QStringLiteral("重新")) || text.contains(QStringLiteral("retry"), Qt::CaseInsensitive))
		{
			resetFeatureSession();
			if (m_dock)
			{
				m_dock->appendSystemMessage(QStringLiteral("已重置特征识别会话，请重新描述要识别的特征。"));
			}
			return true;
		}
		std::vector<std::string> selectedIds;
		QString selErr;
		if (!parseDisplayIndexSelectionLocal(text, m_pendingCatalogSliceUtf8, selectedIds, &selErr))
		{
			return false;
		}
		const QByteArray planJson = buildMinimalFeaturePlanFromSelection(
			selectedIds, m_pendingCatalogFullUtf8, m_pendingWorkpieceBackendId, m_pendingWorkpieceStepPath,
			m_pendingPipelineTemplate.isEmpty() ? QStringLiteral("weld_default") : m_pendingPipelineTemplate);
		if (planJson.isEmpty())
		{
			if (m_dock)
			{
				m_dock->appendAssistantMessage(QStringLiteral("无法将编号映射到特征。"));
			}
			return true;
		}
		m_pendingFeaturePlanJson = planJson;
		m_featureSessionState = FeatureSessionState::AwaitingSelection;
		const QByteArray selectedSlice = filterCatalogSliceByCandidateIds(m_pendingCatalogSliceUtf8, selectedIds);
		if (m_dock)
		{
			m_dock->showTrajectoryFeatureResult(planJson, selectedSlice, QStringLiteral("Selection"));
			beginUnifiedDomainConfirm(AiAgentConfirmKind::TrajectoryCommit, planJson, QStringLiteral("确认并离散"),
									  QStringLiteral("确认并离散"), QStringLiteral("重新识别"),
									  QStringLiteral("Selection"));
		}
		if (m_pluginHost)
		{
			(void)m_pluginHost->showAiFeatureCandidatePreview(selectedSlice, nullptr);
		}
		return true;
	}

	return false;
}

void AiAssistantCoordinator::handleTrajectoryParseResult(const AiParseResult& result)
{
	if (!m_dock)
	{
		return;
	}

	QString clarify;
	if (planIsAmbiguous(result.outputJsonUtf8, &clarify))
	{
		if (isCatalogEmptyTrajectoryMessage(clarify) && !m_trajCatalogRetryUsed && !m_lastTrajectoryUserText.isEmpty())
		{
			scheduleTrajectoryCatalogRetry(m_lastTrajectoryUserText);
			return;
		}
		m_featureSessionState = FeatureSessionState::AwaitingAxisClarify;
		m_pendingFeatureParserVia = result.parserVia;
		const QString msg = clarify.isEmpty()
								? QStringLiteral("请说明需要线特征（边/焊缝/轮廓）还是面特征（平面/打磨栅格）。")
								: clarify;
		m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, msg));
		m_dock->hideTrajectoryFeatureConfirmButtons();
		emit assistantFinished(msg, false, result.parserVia);
		return;
	}

	m_pendingFeaturePlanJson = result.outputJsonUtf8;
	m_pendingFeatureParserVia = result.parserVia;
	m_featureSessionState = FeatureSessionState::PreviewCandidates;

	try
	{
		const nlohmann::json j = nlohmann::json::parse(result.outputJsonUtf8.constData(), nullptr, true);
		m_pendingPipelineTemplate =
			QString::fromStdString(j.value("suggestedPipelineTemplate", std::string("weld_default")));
	}
	catch (...)
	{
		m_pendingPipelineTemplate = QStringLiteral("weld_default");
	}

	if (m_pluginHost)
	{
		(void)m_pluginHost->showAiFeatureCandidatePreview(m_pendingCatalogSliceUtf8, nullptr);
	}

	m_dock->showTrajectoryFeatureResult(result.outputJsonUtf8, m_pendingCatalogSliceUtf8, result.parserVia);
	beginUnifiedDomainConfirm(AiAgentConfirmKind::TrajectoryCommit, result.outputJsonUtf8, QStringLiteral("确认并离散"),
							  QStringLiteral("确认并离散"), QStringLiteral("重新识别"), result.parserVia);
	emit assistantFinished(QStringLiteral("Feature candidates ready for confirmation."), false, result.parserVia);
}

void AiAssistantCoordinator::scheduleTrajectoryCatalogRetry(const QString& userText)
{
	if (m_trajCatalogRetryUsed || userText.isEmpty())
	{
		return;
	}
	m_trajCatalogRetryUsed = true;
	QTimer::singleShot(0, this, [this, userText]() { retryTrajectoryFeatureWithRules(userText); });
}

void AiAssistantCoordinator::retryTrajectoryFeatureWithRules(const QString& userText)
{
	if (!m_dock || !m_aiHost || !m_pluginHost)
	{
		return;
	}

	m_dock->appendSystemMessage(QStringLiteral("特征目录为空或未传给 LLM，正在重新枚举并以规则解析重试…"));
	m_dock->setBusy(true);

	AiInferenceRequest req;
	req.domainId = AiDomainIds::trajectoryFeature();
	req.userText = userText;
	QString prepErr;
	if (!prepareTrajectoryFeatureRequest(userText, req, &prepErr))
	{
		m_dock->setBusy(false);
		m_dock->appendAssistantMessage(prepErr);
		emit parseFailed(prepErr, QStringLiteral("Rules"));
		return;
	}

	m_pendingWorkpieceBackendId = req.workpieceBackendId;
	m_pendingWorkpieceStepPath = req.workpieceStepPathUtf8;
	m_pendingCatalogFullUtf8 = req.catalogFullUtf8;
	m_pendingCatalogSliceUtf8 = req.catalogSliceUtf8;
	m_pendingFeatureAxis = inferFeatureAxisLocal(userText);

	if (!catalogSliceHasCandidates(req.catalogSliceUtf8))
	{
		m_dock->setBusy(false);
		const QString msg =
			QStringLiteral("当前 STEP 工件未找到匹配的线/面特征候选，请确认模型已加载且轨迹页已选工件。");
		m_dock->appendAssistantMessage(msg);
		emit parseFailed(msg, QStringLiteral("Rules"));
		return;
	}

	const AiParseResult rules = m_aiHost->parseTrajectoryFeatureRequest(req);
	m_dock->setBusy(false);
	if (!rules.ok)
	{
		QString msg = rules.errorMessage;
		if (msg.isEmpty())
		{
			msg = QStringLiteral("规则解析失败。");
		}
		m_dock->appendAssistantMessage(prefixWithParser(rules.parserVia, msg));
		emit parseFailed(msg, rules.parserVia);
		return;
	}

	handleTrajectoryParseResult(rules);
}

bool AiAssistantCoordinator::needsViewportCapture(const QString& domainId, const QString& userText,
												  const AiConfigDto& cfg) const
{
	const QString resolved = m_aiHost ? m_aiHost->resolveDomainId(domainId, userText) : domainId.trimmed();
	if (resolved == AiDomainIds::geometryRecognize())
		return true;
	if (const AiDomainModelConfig* dm = findDomainConfig(cfg, resolved))
		return dm->multimodal;
	return false;
}

void AiAssistantCoordinator::onUserMessageSubmitted(const QString& text)
{
	if (!m_dock)
		return;
	if (!m_aiHost)
	{
		const QString msg =
			QStringLiteral("AI 宿主未就绪（插件宿主尚未初始化）。请重新编译并启动 CloudSim，或稍后重试。");
		m_dock->appendAssistantMessage(msg);
		emit parseFailed(msg, QString());
		return;
	}

	if (tryHandleFeatureFollowUp(text))
	{
		return;
	}

	m_trajCatalogRetryUsed = false;
	m_lastTrajectoryUserText.clear();

	if (m_aiHost)
		m_aiHost->cancelAgentTurn();

	m_dock->setBusy(true);
	m_dock->hideCreateFromRecognitionButton();
	m_dock->hideTrajectoryFeatureConfirmButtons();
	m_dock->hideAgentConfirmPanel();
	m_pendingRecognitionJson.clear();
	m_pendingRecognitionParserVia.clear();

	const std::optional<AiConfigDto> cfgOpt = m_aiHost->loadConfig();
	const AiConfigDto cfg = cfgOpt ? *cfgOpt : defaultAiConfigDto();

	AiInferenceRequest req;
	req.domainId = m_dock->selectedDomainId();
	req.userText = text;

	const QString resolvedDomain = m_aiHost->resolveDomainId(req.domainId, text);

	if (shouldUseAgentRuntime(resolvedDomain))
	{
		startAgentTurn(text, req.domainId);
		return;
	}

	if (resolvedDomain == AiDomainIds::trajectoryFeature())
	{
		m_lastTrajectoryUserText = text;
		QString prepErr;
		if (!prepareTrajectoryFeatureRequest(text, req, &prepErr))
		{
			m_dock->setBusy(false);
			m_dock->appendAssistantMessage(prepErr);
			emit parseFailed(prepErr, QString());
			return;
		}
		m_pendingWorkpieceBackendId = req.workpieceBackendId;
		m_pendingWorkpieceStepPath = req.workpieceStepPathUtf8;
		m_pendingCatalogFullUtf8 = req.catalogFullUtf8;
		m_pendingCatalogSliceUtf8 = req.catalogSliceUtf8;
		m_pendingFeatureAxis = inferFeatureAxisLocal(text);
	}

	if (needsViewportCapture(req.domainId, text, cfg))
	{
		if (!m_pluginHost)
		{
			m_dock->setBusy(false);
			const QString msg = QStringLiteral("宿主上下文未就绪，无法截取视口。");
			m_dock->appendAssistantMessage(msg);
			emit parseFailed(msg, QString());
			return;
		}
		QString capErr;
		if (!m_pluginHost->captureActiveViewportPng(req.imagePng, &capErr))
		{
			m_dock->setBusy(false);
			const QString msg =
				capErr.isEmpty() ? QStringLiteral("无法截取当前 3D 视口，请先打开含视口的文档。") : capErr;
			m_dock->appendAssistantMessage(msg);
			emit parseFailed(msg, QString());
			return;
		}
	}

	m_aiHost->parseUserTextAsync(
		req, cfg,
		[this](double fraction, const QString& message)
		{
			if (m_dock && !message.isEmpty())
				m_dock->appendSystemMessage(
					QStringLiteral("%1% — %2").arg(static_cast<int>(fraction * 100)).arg(message));
		},
		[this, resolvedDomain, text](AiParseResult result)
		{
			if (!m_dock)
				return;
			m_dock->setBusy(false);
			if (!result.ok)
			{
				if (resolvedDomain == AiDomainIds::trajectoryFeature() &&
					(isCatalogEmptyTrajectoryMessage(result.errorMessage) ||
					 !catalogSliceHasCandidates(m_pendingCatalogSliceUtf8)) &&
					!m_trajCatalogRetryUsed && !m_lastTrajectoryUserText.isEmpty())
				{
					scheduleTrajectoryCatalogRetry(m_lastTrajectoryUserText);
					return;
				}
				QString msg = result.errorMessage;
				if (!result.hintMessage.isEmpty())
					msg += QStringLiteral("\n") + result.hintMessage;
				m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, msg));
				emit parseFailed(msg, result.parserVia);
				return;
			}

			if (result.domainId == AiDomainIds::geometryRecognize())
			{
				m_pendingRecognitionJson = result.outputJsonUtf8;
				m_pendingRecognitionParserVia = result.parserVia;
				m_dock->showRecognitionResult(result.outputJsonUtf8, result.parserVia);
				nlohmann::json j;
				try
				{
					j = nlohmann::json::parse(result.outputJsonUtf8.constData(), nullptr, true);
				}
				catch (...)
				{
					j = nlohmann::json::object();
				}
				const std::string prim = j.value("primitive", std::string());
				if (prim != "unknown" && !prim.empty())
				{
					beginUnifiedDomainConfirm(AiAgentConfirmKind::RecognizeCreate, result.outputJsonUtf8,
											  QStringLiteral("确认创建基本体"), QStringLiteral("确认创建"), QString(),
											  result.parserVia);
				}
				emit assistantFinished(QStringLiteral("Recognition complete."), false, result.parserVia);
				return;
			}

			if (result.domainId == AiDomainIds::trajectoryFeature())
			{
				handleTrajectoryParseResult(result);
				return;
			}

			QString summary;
			QString err;
			bool executed = false;
			if (result.outputKind == AiDomainOutputKind::ActionPlan)
				executed = m_aiHost->executeActionPlan(result.outputJsonUtf8, &summary, &err);
			else
				executed = m_aiHost->executeDomainOutput(result.domainId, result.outputJsonUtf8, &summary, &err);

			if (!executed)
			{
				const QString msg = err.isEmpty() ? QStringLiteral("Failed to execute AI plan.") : err;
				m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, msg));
				emit parseFailed(msg, result.parserVia);
				return;
			}

			QString reply = summary.isEmpty() ? QStringLiteral("Done.") : summary;
			if (!result.hintMessage.isEmpty())
				reply += QStringLiteral("\n") + result.hintMessage;
			m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, reply));
			emit assistantFinished(reply, false, result.parserVia);
		});
}

void AiAssistantCoordinator::onCreateRecognitionConfirmed()
{
	if (!m_dock || !m_aiHost || m_pendingRecognitionJson.isEmpty())
		return;

	m_dock->setBusy(true);
	QString summary;
	QString err;
	const bool executed =
		m_aiHost->executeDomainOutput(AiDomainIds::geometryRecognize(), m_pendingRecognitionJson, &summary, &err);
	m_dock->setBusy(false);

	if (!executed)
	{
		const QString msg = err.isEmpty() ? QStringLiteral("创建基本体失败。") : err;
		m_dock->appendAssistantMessage(prefixWithParser(m_pendingRecognitionParserVia, msg));
		emit parseFailed(msg, m_pendingRecognitionParserVia);
		return;
	}

	m_dock->hideCreateFromRecognitionButton();
	m_dock->hideAgentConfirmPanel();
	m_pendingRecognitionJson.clear();
	const QString reply = summary.isEmpty() ? QStringLiteral("基本体已创建。") : summary;
	m_dock->appendAssistantMessage(prefixWithParser(m_pendingRecognitionParserVia, reply));
	emit assistantFinished(reply, false, m_pendingRecognitionParserVia);
}

void AiAssistantCoordinator::onConfirmTrajectoryFeaturesClicked()
{
	if (!m_dock || !m_pluginHost || m_pendingFeaturePlanJson.isEmpty())
	{
		return;
	}
	m_dock->setBusy(true);
	QString summary;
	QString err;
	const bool ok = m_pluginHost->commitAiTrajectoryFeatures(m_pendingFeaturePlanJson, &summary, &err);
	m_dock->setBusy(false);
	if (!ok)
	{
		const QString msg = err.isEmpty() ? QStringLiteral("离散/提交失败") : err;
		m_dock->appendAssistantMessage(prefixWithParser(m_pendingFeatureParserVia, msg));
		emit parseFailed(msg, m_pendingFeatureParserVia);
		return;
	}
	m_dock->hideTrajectoryFeatureConfirmButtons();
	m_dock->hideAgentConfirmPanel();
	resetFeatureSession();
	const QString reply = summary.isEmpty() ? QStringLiteral("轨迹特征已离散并填充默认流水线。") : summary;
	m_dock->appendAssistantMessage(prefixWithParser(m_pendingFeatureParserVia, reply));
	emit assistantFinished(reply, false, m_pendingFeatureParserVia);
}

void AiAssistantCoordinator::onRetryTrajectoryFeaturesClicked()
{
	resetFeatureSession();
	if (m_dock)
	{
		m_dock->appendSystemMessage(QStringLiteral("已取消当前特征候选，请重新输入识别指令。"));
	}
}

bool AiAssistantCoordinator::shouldUseAgentRuntime(const QString& resolvedDomainId) const
{
	return resolvedDomainId == AiDomainIds::meshCreate() || resolvedDomainId == AiDomainIds::meshCompose() ||
		   resolvedDomainId == AiDomainIds::pointCloudOps() || resolvedDomainId == AiDomainIds::documentImport() ||
		   resolvedDomainId == AiDomainIds::geometryOps() || resolvedDomainId == AiDomainIds::featureBuild() ||
		   resolvedDomainId == AiDomainIds::labelingAnnot() || resolvedDomainId == AiDomainIds::sceneOps();
}

void AiAssistantCoordinator::startAgentTurn(const QString& text, const QString& domainId)
{
	if (!m_dock || !m_aiHost)
		return;
	const std::optional<AiConfigDto> cfgOpt = m_aiHost->loadConfig();
	const AiConfigDto cfg = cfgOpt ? *cfgOpt : defaultAiConfigDto();
	AiInferenceRequest req;
	req.domainId = domainId;
	req.userText = text;
	m_dock->setBusy(true);
	m_aiHost->runAgentTurnAsync(
		req, cfg,
		[this](double fraction, const QString& message)
		{
			if (m_dock && !message.isEmpty())
				m_dock->appendSystemMessage(
					QStringLiteral("%1% — %2").arg(static_cast<int>(fraction * 100)).arg(message));
		},
		[this](const AiAgentEvent& ev) { handleAgentEvent(ev); });
}

void AiAssistantCoordinator::beginUnifiedDomainConfirm(AiAgentConfirmKind kind, const QByteArray& payload,
													   const QString& title, const QString& confirmLabel,
													   const QString& secondaryLabel, const QString& parserVia)
{
	if (!m_dock || !m_aiHost)
		return;
	AiDomainConfirmRequest req;
	req.kind = kind;
	req.payloadUtf8 = payload;
	req.title = title;
	req.risk = QStringLiteral("medium");
	req.confirmLabel = confirmLabel;
	req.secondaryLabel = secondaryLabel;
	req.parserVia = parserVia;
	m_aiHost->beginDomainConfirmAsync(req, [this](const AiAgentEvent& ev) { handleAgentEvent(ev); });
}

void AiAssistantCoordinator::handleAgentEvent(const AiAgentEvent& ev)
{
	if (!m_dock)
		return;
	switch (ev.kind)
	{
	case AiAgentEventKind::NeedConfirm:
		m_dock->setBusy(false);
		m_dock->appendSystemMessage(ev.message.isEmpty() ? QStringLiteral("请确认参数后执行。") : ev.message);
		m_dock->showAgentConfirmPanel(ev.pendingId, ev.title, ev.risk, ev.argsSchemaJson, ev.proposedArgsJson,
									  ev.sceneSnapshotJson, ev.confirmLabel, ev.secondaryLabel);
		break;
	case AiAgentEventKind::StepDone:
		m_dock->appendAssistantMessage(ev.message);
		m_dock->setBusy(true);
		break;
	case AiAgentEventKind::Finished:
		m_dock->hideAgentConfirmPanel();
		m_dock->setBusy(false);
		m_dock->appendAssistantMessage(ev.message.isEmpty() ? QStringLiteral("完成。") : ev.message);
		if (ev.toolId == QStringLiteral("geometry.recognize.create") ||
			ev.confirmKind == AiAgentConfirmKind::RecognizeCreate)
			m_pendingRecognitionJson.clear();
		if (ev.toolId == QStringLiteral("trajectory.feature.commit") ||
			ev.confirmKind == AiAgentConfirmKind::TrajectoryCommit)
			resetFeatureSession();
		emit assistantFinished(ev.message, false, QStringLiteral("Agent"));
		break;
	case AiAgentEventKind::Error:
		m_dock->hideAgentConfirmPanel();
		m_dock->setBusy(false);
		m_dock->appendAssistantMessage(ev.message);
		emit parseFailed(ev.message, QStringLiteral("Agent"));
		break;
	case AiAgentEventKind::Secondary:
		m_dock->hideAgentConfirmPanel();
		m_dock->setBusy(false);
		if (ev.toolId == QStringLiteral("trajectory.feature.commit"))
			onRetryTrajectoryFeaturesClicked();
		break;
	}
}

void AiAssistantCoordinator::onAgentConfirmAccepted(const QString& pendingId, const QByteArray& argsJsonUtf8)
{
	if (!m_aiHost)
		return;
	m_dock->setBusy(true);
	m_aiHost->submitAgentConfirm(pendingId, argsJsonUtf8);
}

void AiAssistantCoordinator::onAgentConfirmRejected(const QString& pendingId)
{
	if (!m_aiHost)
		return;
	m_aiHost->cancelAgentConfirm(pendingId);
}

void AiAssistantCoordinator::onAgentConfirmSecondary(const QString& pendingId)
{
	if (!m_aiHost)
		return;
	m_aiHost->secondaryAgentConfirm(pendingId);
}
