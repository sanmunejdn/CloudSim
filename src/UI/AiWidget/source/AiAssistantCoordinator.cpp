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

#include <QRegularExpression>
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

bool looksLikeFeatureSelectionAttempt(const QString& userText)
{
	const QString t = userText.trimmed();
	if (t.isEmpty())
		return false;
	if (t.contains(QStringLiteral("选")) || t.contains(QStringLiteral("选择")))
		return true;
	static const QRegularExpression idRe(QStringLiteral(R"((?i)\b(?:edge|face|seam|weld)_\d+\b)"));
	if (idRe.match(t).hasMatch())
		return true;
	// 纯数字短句（如「14」「1 和 3」）
	bool anyDigit = false;
	for (const QChar c : t)
	{
		if (c.isDigit())
			anyDigit = true;
		else if (!c.isSpace() && c != QLatin1Char(',') && c != QLatin1Char('、') && c != QLatin1Char('和') &&
				 c != QLatin1Char('-'))
			return false;
	}
	return anyDigit;
}

bool isConfirmDiscretizePhrase(const QString& userText)
{
	const QString t = userText.trimmed();
	if (t.isEmpty())
		return false;
	if (t.compare(QStringLiteral("确认"), Qt::CaseInsensitive) == 0 ||
		t.compare(QStringLiteral("ok"), Qt::CaseInsensitive) == 0 ||
		t.compare(QStringLiteral("yes"), Qt::CaseInsensitive) == 0)
		return true;
	return t.contains(QStringLiteral("确认并离散")) || t.contains(QStringLiteral("确认离散")) ||
		   t.contains(QStringLiteral("开始离散")) || t.contains(QStringLiteral("确认执行"));
}

bool isReRecognizePhrase(const QString& userText)
{
	const QString t = userText.trimmed();
	return t.contains(QStringLiteral("重新识别")) || t.contains(QStringLiteral("再识别")) ||
		   t.compare(QStringLiteral("重新开始识别"), Qt::CaseInsensitive) == 0 ||
		   t.compare(QStringLiteral("retry"), Qt::CaseInsensitive) == 0;
}

bool isCancelFeatureSessionPhrase(const QString& userText)
{
	const QString t = userText.trimmed();
	return t == QStringLiteral("取消") || t == QStringLiteral("放弃") || t == QStringLiteral("关闭候选") ||
		   t.compare(QStringLiteral("cancel"), Qt::CaseInsensitive) == 0;
}

bool isNewFeatureRecognizeIntent(const QString& userText)
{
	const QString t = userText.trimmed();
	return t.contains(QStringLiteral("线特征")) || t.contains(QStringLiteral("面特征")) ||
		   t.contains(QStringLiteral("识别焊缝")) || t.contains(QStringLiteral("识别边")) ||
		   t.contains(QStringLiteral("识别打磨")) || t.contains(QStringLiteral("打磨面")) ||
		   t.contains(QStringLiteral("轨迹特征"));
}

bool parseDisplayIndexSelectionLocal(const QString& userText, const QByteArray& catalogSliceUtf8,
									 const QByteArray& catalogFullUtf8, std::vector<std::string>& outCandidateIds,
									 QString* err)
{
	outCandidateIds.clear();
	std::set<std::string> idHits;
	std::set<int> indices;
	const QString t = userText;

	static const QRegularExpression idRe(QStringLiteral(R"((?i)\b((?:edge|face|seam|weld)_\d+)\b)"));
	QRegularExpressionMatchIterator it = idRe.globalMatch(t);
	QString tForDigits = t;
	while (it.hasNext())
	{
		const QRegularExpressionMatch m = it.next();
		idHits.insert(m.captured(1).toLower().toStdString());
		// 避免把 face_13 里的 13 再当 displayIndex
		tForDigits.replace(m.captured(0), QStringLiteral(" "));
	}

	for (int i = 0; i < tForDigits.size(); ++i)
	{
		if (tForDigits[i].isDigit())
		{
			int val = 0;
			while (i < tForDigits.size() && tForDigits[i].isDigit())
			{
				val = val * 10 + tForDigits[i].digitValue();
				++i;
			}
			if (val > 0)
				indices.insert(val);
			--i;
		}
	}

	if (idHits.empty() && indices.empty())
	{
		if (err)
			*err = QStringLiteral("未解析到编号，请使用如「选 1 和 3」或「选 face_13」。");
		return false;
	}

	int maxDisplay = 0;
	std::set<std::string> resolvedIds;
	try
	{
		const nlohmann::json j = nlohmann::json::parse(catalogSliceUtf8.constData(), nullptr, true);
		std::set<std::string> seen;
		for (const auto& item : j["candidates"])
		{
			const int displayIndex = item.value("displayIndex", 0);
			if (displayIndex > maxDisplay)
				maxDisplay = displayIndex;
			const std::string cid = item.value("candidateId", std::string());
			const std::string cidLower = QString::fromStdString(cid).toLower().toStdString();
			const bool byId = !cidLower.empty() && idHits.count(cidLower) > 0;
			const bool byIndex = indices.count(displayIndex) > 0;
			if ((byId || byIndex) && seen.insert(cid).second)
			{
				outCandidateIds.push_back(cid);
				resolvedIds.insert(cidLower);
			}
		}
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("候选列表解析失败。");
		return false;
	}

	// 切片未收录时，仍可用 face_63 等 ID 从全量目录解析
	if (!idHits.empty() && !catalogFullUtf8.isEmpty())
	{
		try
		{
			const nlohmann::json full = nlohmann::json::parse(catalogFullUtf8.constData(), nullptr, true);
			for (const auto& item : full["candidates"])
			{
				const std::string cid = item.value("candidateId", std::string());
				const std::string cidLower = QString::fromStdString(cid).toLower().toStdString();
				if (cidLower.empty() || idHits.count(cidLower) == 0 || resolvedIds.count(cidLower) > 0)
					continue;
				outCandidateIds.push_back(cid);
				resolvedIds.insert(cidLower);
			}
		}
		catch (...)
		{
		}
	}

	if (outCandidateIds.empty())
	{
		if (err)
		{
			if (!idHits.empty())
				*err = QStringLiteral("未找到对应候选 ID，请对照列表中的 edge_/face_ 编号，或输入「选 face_63」。");
			else if (maxDisplay > 0)
				*err = QStringLiteral("编号超出范围（当前 1–%1），请重新选择。").arg(maxDisplay);
			else
				*err = QStringLiteral("编号超出当前候选范围。");
		}
		return false;
	}
	return true;
}

QByteArray filterCatalogSliceByCandidateIds(const QByteArray& catalogSliceUtf8, const QByteArray& catalogFullUtf8,
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
		std::set<std::string> idSet(candidateIds.begin(), candidateIds.end());
		std::set<std::string> found;
		auto appendFrom = [&](const nlohmann::json& catalog)
		{
			if (!catalog.contains("candidates") || !catalog["candidates"].is_array())
				return;
			for (const auto& c : catalog["candidates"])
			{
				const std::string id = c.value("candidateId", std::string());
				if (idSet.count(id) == 0 || found.count(id) > 0)
					continue;
				found.insert(id);
				nlohmann::json row = c;
				if (!row.contains("displayIndex"))
					row["displayIndex"] = static_cast<int>(found.size());
				dst["candidates"].push_back(row);
			}
		};
		appendFrom(src);
		if (found.size() < idSet.size() && !catalogFullUtf8.isEmpty())
		{
			const nlohmann::json full = nlohmann::json::parse(catalogFullUtf8.constData(), nullptr, true);
			appendFrom(full);
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
		plan["version"] = 2;
		plan["schemaVersion"] = 2;
		plan["selectedCandidateIds"] = candidateIds;
		plan["suggestedPipelineTemplate"] = pipelineTemplate.toStdString();
		plan["workpiece"] = {
			{"backendIdUtf8", backendId.toStdString()},
			{"stepPathUtf8", stepPath.toStdString()},
		};
		nlohmann::json feats = nlohmann::json::array();
		for (const std::string& id : candidateIds)
		{
			for (const auto& c : full["candidates"])
			{
				if (c.value("candidateId", std::string()) != id)
					continue;
				// catalog 字段是 suggestedStrategyId + geometry；旧键 suggestedKind/refs 会导致面特征落成空 EdgeChain
				const std::string strategyId =
					c.value("suggestedStrategyId", c.value("suggestedKind", std::string("EdgeChain")));
				nlohmann::json spec;
				spec["schemaVersion"] = 1;
				spec["featureId"] = id;
				spec["strategyId"] = strategyId;
				spec["kind"] = strategyId;
				spec["workpiece"] = plan["workpiece"];
				if (c.contains("geometry") && c["geometry"].is_object())
					spec["geometry"] = c["geometry"];
				else if (c.contains("refs") && c["refs"].is_object())
					spec["refs"] = c["refs"];
				// params 由宿主 enrichTrajectoryPlanJsonInPlace 按策略默认补全
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
	if (!m_pluginHost)
		return false;

	if (m_featureSessionState == FeatureSessionState::Idle)
	{
		if (isReRecognizePhrase(text))
		{
			if (m_lastTrajectoryUserText.isEmpty())
			{
				if (m_dock)
				{
					m_dock->appendAssistantMessage(
						QStringLiteral("暂无上次识别指令。请说「线特征识别」或「面特征识别」。"));
				}
				return true;
			}
			return rerunTrajectoryFeatureRecognize(m_lastTrajectoryUserText);
		}
		if (looksLikeFeatureSelectionAttempt(text) || isConfirmDiscretizePhrase(text))
		{
			if (m_dock)
			{
				m_dock->appendAssistantMessage(
					QStringLiteral("当前没有特征候选。请先「线特征识别」或「面特征识别」，或「重新识别」。"));
			}
			return true;
		}
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
		if (isCancelFeatureSessionPhrase(text))
		{
			resetFeatureSession();
			if (m_dock)
				m_dock->appendSystemMessage(QStringLiteral("已关闭特征候选。可再说「线特征识别」或「重新识别」。"));
			return true;
		}
		if (isReRecognizePhrase(text))
		{
			const QString again =
				m_lastTrajectoryUserText.isEmpty() ? QStringLiteral("面特征识别") : m_lastTrajectoryUserText;
			return rerunTrajectoryFeatureRecognize(again);
		}
		if (isNewFeatureRecognizeIntent(text))
			return false;

		if (isConfirmDiscretizePhrase(text))
		{
			if (m_pendingFeaturePlanJson.isEmpty())
			{
				if (m_dock)
				{
					m_dock->appendAssistantMessage(
						QStringLiteral("请先输入「选 N」选定特征，再「确认」打开离散对话框。"));
				}
				return true;
			}
			try
			{
				const nlohmann::json j =
					nlohmann::json::parse(m_pendingFeaturePlanJson.constData(), nullptr, true);
				if (!j.contains("features") || !j["features"].is_array() || j["features"].empty())
				{
					if (m_dock)
					{
						m_dock->appendAssistantMessage(
							QStringLiteral("当前计划尚无特征，请先「选 N」再确认。"));
					}
					return true;
				}
			}
			catch (...)
			{
				if (m_dock)
					m_dock->appendAssistantMessage(QStringLiteral("特征计划无效，请重新选择。"));
				return true;
			}
			beginUnifiedDomainConfirm(AiAgentConfirmKind::TrajectoryCommit, m_pendingFeaturePlanJson,
									  QStringLiteral("确认并离散"), QStringLiteral("确认并离散"),
									  QStringLiteral("返回重选"), m_pendingFeatureParserVia);
			return true;
		}

		if (looksLikeFeatureSelectionAttempt(text))
		{
			std::vector<std::string> selectedIds;
			QString selErr;
			if (!parseDisplayIndexSelectionLocal(text, m_pendingCatalogSliceUtf8, m_pendingCatalogFullUtf8, selectedIds,
												&selErr))
			{
				if (m_dock)
					m_dock->appendAssistantMessage(selErr);
				return true;
			}
			const QByteArray planJson = buildMinimalFeaturePlanFromSelection(
				selectedIds, m_pendingCatalogFullUtf8, m_pendingWorkpieceBackendId, m_pendingWorkpieceStepPath,
				m_pendingPipelineTemplate.isEmpty() ? QStringLiteral("weld_default") : m_pendingPipelineTemplate);
			if (planJson.isEmpty())
			{
				if (m_dock)
					m_dock->appendAssistantMessage(QStringLiteral("无法将编号映射到特征。"));
				return true;
			}
			m_pendingFeaturePlanJson = planJson;
			m_featureSessionState = FeatureSessionState::AwaitingSelection;
			const QByteArray selectedSlice = filterCatalogSliceByCandidateIds(
				m_pendingCatalogSliceUtf8, m_pendingCatalogFullUtf8, selectedIds);
			if (m_dock)
			{
				m_dock->showTrajectoryFeatureResult(planJson, selectedSlice, QStringLiteral("Selection"));
				m_dock->appendSystemMessage(
					QStringLiteral("已更新选择。输入「确认」或「确认并离散」打开对话框核对策略与算子；"
								   "也可继续「选 N」调整。"));
			}
			if (m_pluginHost)
				(void)m_pluginHost->showAiFeatureCandidatePreview(selectedSlice, nullptr);
			return true;
		}

		return false;
	}

	return false;
}

bool AiAssistantCoordinator::rerunTrajectoryFeatureRecognize(const QString& userText)
{
	if (!m_dock || !m_aiHost || !m_pluginHost || userText.trimmed().isEmpty())
		return false;

	resetFeatureSession();
	m_trajCatalogRetryUsed = false;
	m_lastTrajectoryUserText = userText.trimmed();
	m_dock->setBusy(true);
	m_dock->hideAgentConfirmPanel();
	m_dock->appendSystemMessage(QStringLiteral("正在按「%1」重新识别特征…").arg(m_lastTrajectoryUserText));

	AiInferenceRequest req;
	req.domainId = AiDomainIds::trajectoryFeature();
	req.userText = m_lastTrajectoryUserText;
	QString prepErr;
	if (!prepareTrajectoryFeatureRequest(m_lastTrajectoryUserText, req, &prepErr))
	{
		m_dock->setBusy(false);
		m_dock->appendAssistantMessage(prepErr);
		emit parseFailed(prepErr, QStringLiteral("Rules"));
		return true;
	}

	m_pendingWorkpieceBackendId = req.workpieceBackendId;
	m_pendingWorkpieceStepPath = req.workpieceStepPathUtf8;
	m_pendingCatalogFullUtf8 = req.catalogFullUtf8;
	m_pendingCatalogSliceUtf8 = req.catalogSliceUtf8;
	m_pendingFeatureAxis = inferFeatureAxisLocal(m_lastTrajectoryUserText);

	const AiParseResult rules = m_aiHost->parseTrajectoryFeatureRequest(req);
	m_dock->setBusy(false);
	if (!rules.ok)
	{
		m_dock->appendAssistantMessage(prefixWithParser(rules.parserVia, rules.errorMessage));
		emit parseFailed(rules.errorMessage, rules.parserVia);
		return true;
	}
	handleTrajectoryParseResult(rules);
	return true;
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
	m_dock->appendSystemMessage(
		QStringLiteral("请输入「选 N」选定特征；选好后输入「确认」打开离散对话框，或继续调整选择。"));
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

	if (tryHandleTrajectoryPlanRevise(text))
	{
		return;
	}

	if (tryHandleFeatureFollowUp(text))
	{
		return;
	}

	m_trajCatalogRetryUsed = false;
	// 保留 m_lastTrajectoryUserText，供 Idle「重新识别」复用

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

	// 切到非轨迹域时结束特征候选会话（仍保留上次识别原文）
	if (!resolvedDomain.isEmpty() && resolvedDomain != AiDomainIds::trajectoryFeature() &&
		m_featureSessionState != FeatureSessionState::Idle)
	{
		resetFeatureSession();
	}

	const bool domainLocked = !req.domainId.isEmpty() && req.domainId != AiDomainIds::autoDomain();
	if (!domainLocked && resolvedDomain.isEmpty())
	{
		m_dock->setBusy(false);
		const QString tip = QStringLiteral(
			"未识别到明确意图，未执行任何操作。\n"
			"请先在上方选择领域，或说得更具体，例如：\n"
			"· 生成长方体 / 生成圆柱\n"
			"· 建模 100x100x100 通孔 d50\n"
			"· 线特征识别 / 面特征识别 / 重新识别\n"
			"· 体素下采样 / 点云匹配\n"
			"· 修改离散参数（已有轨迹时）");
		m_dock->appendAssistantMessage(tip);
		emit assistantFinished(tip, false, QStringLiteral("clarify"));
		return;
	}

	if (shouldUseAgentRuntime(resolvedDomain))
	{
		startAgentTurn(text, domainLocked ? req.domainId : resolvedDomain);
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
	beginUnifiedDomainConfirm(AiAgentConfirmKind::TrajectoryCommit, m_pendingFeaturePlanJson,
							  QStringLiteral("确认并离散"), QStringLiteral("确认并离散"),
							  QStringLiteral("返回重选"), m_pendingFeatureParserVia);
}

void AiAssistantCoordinator::onRetryTrajectoryFeaturesClicked()
{
	// Dock 次按钮：关闭候选；对话框「返回重选」走 Outcome::Retry，不经此路径清会话
	resetFeatureSession();
	if (m_dock)
	{
		m_dock->appendSystemMessage(
			QStringLiteral("已取消当前特征候选。可输入「重新识别」复用上次指令，或再说「线/面特征识别」。"));
	}
}

void AiAssistantCoordinator::restoreTrajectoryCandidatePreview()
{
	if (!m_pluginHost)
		return;
	if (m_featureSessionState == FeatureSessionState::Idle)
		m_featureSessionState = FeatureSessionState::AwaitingSelection;
	if (!m_pendingFeaturePlanJson.isEmpty())
	{
		try
		{
			const nlohmann::json j = nlohmann::json::parse(m_pendingFeaturePlanJson.constData(), nullptr, true);
			std::vector<std::string> ids;
			if (j.contains("selectedCandidateIds") && j["selectedCandidateIds"].is_array())
			{
				for (const auto& id : j["selectedCandidateIds"])
				{
					if (id.is_string())
						ids.push_back(id.get<std::string>());
				}
			}
			const QByteArray slice =
				ids.empty() ? m_pendingCatalogSliceUtf8
							: filterCatalogSliceByCandidateIds(m_pendingCatalogSliceUtf8, m_pendingCatalogFullUtf8, ids);
			(void)m_pluginHost->showAiFeatureCandidatePreview(slice, nullptr);
		}
		catch (...)
		{
			(void)m_pluginHost->showAiFeatureCandidatePreview(m_pendingCatalogSliceUtf8, nullptr);
		}
	}
	else if (!m_pendingCatalogSliceUtf8.isEmpty())
	{
		(void)m_pluginHost->showAiFeatureCandidatePreview(m_pendingCatalogSliceUtf8, nullptr);
	}
}

void AiAssistantCoordinator::openTrajectoryDiscretizeDialog(const QString& pendingId, const QByteArray& planIn,
															const bool showRetry)
{
	if (!m_dock || !m_pluginHost || !m_aiHost || planIn.isEmpty() || pendingId.isEmpty())
		return;
	m_dock->setBusy(false);
	m_dock->hideTrajectoryFeatureConfirmButtons();
	m_dock->hideAgentConfirmPanel();
	QByteArray merged;
	QString err;
	const int code = m_pluginHost->proposeAndConfirmTrajectoryPlan(planIn, merged, &err, showRetry);
	if (code == 2)
	{
		m_aiHost->secondaryAgentConfirm(pendingId);
		return;
	}
	if (code != 1 || merged.isEmpty())
	{
		m_aiHost->cancelAgentConfirm(pendingId);
		if (m_dock && !err.isEmpty())
			m_dock->appendSystemMessage(err);
		else if (m_dock)
			m_dock->appendSystemMessage(QStringLiteral("已取消离散确认，当前选择仍保留。"));
		return;
	}
	m_dock->setBusy(true);
	m_aiHost->submitAgentConfirm(pendingId, merged);
}

bool AiAssistantCoordinator::tryHandleTrajectoryPlanRevise(const QString& text)
{
	const QString t = text.trimmed();
	if (t.isEmpty() || !m_pluginHost || !m_dock)
		return false;
	// 仅明确短语，避免「修改参数」等口语误弹离散对话框
	const bool hit = t.contains(QStringLiteral("修改离散")) || t.contains(QStringLiteral("改离散参数")) ||
					 t.contains(QStringLiteral("调整离散参数")) || t.contains(QStringLiteral("修改离散参数")) ||
					 t.contains(QStringLiteral("修改管线算子")) || t.contains(QStringLiteral("改管线算子")) ||
					 t.contains(QStringLiteral("修改轨迹算子")) || t.contains(QStringLiteral("重新离散参数"));
	if (!hit)
		return false;
	QByteArray plan;
	QString err;
	if (!m_pluginHost->loadBoundTrajectoryPlanForAi(plan, &err) || plan.isEmpty())
	{
		m_dock->appendAssistantMessage(err.isEmpty() ? QStringLiteral("当前无已绑定的离散结果，请先完成特征离散。")
													 : err);
		emit parseFailed(err, QStringLiteral("revise"));
		return true;
	}
	m_dock->appendSystemMessage(QStringLiteral("请在弹出对话框中修改离散策略、参数或管线算子。"));
	QByteArray merged;
	const int code = m_pluginHost->proposeAndConfirmTrajectoryPlan(plan, merged, &err, false);
	if (code != 1 || merged.isEmpty())
	{
		m_dock->appendSystemMessage(QStringLiteral("已取消修改。"));
		return true;
	}
	m_dock->setBusy(true);
	QString summary;
	QString reviseErr;
	const bool ok = m_pluginHost->reviseAiTrajectoryPlan(merged, &summary, &reviseErr);
	m_dock->setBusy(false);
	if (!ok)
	{
		const QString msg = reviseErr.isEmpty() ? QStringLiteral("更新失败") : reviseErr;
		m_dock->appendAssistantMessage(msg);
		emit parseFailed(msg, QStringLiteral("revise"));
		return true;
	}
	m_dock->appendAssistantMessage(summary.isEmpty() ? QStringLiteral("已更新离散参数与管线算子。") : summary);
	emit assistantFinished(summary, false, QStringLiteral("revise"));
	return true;
}

bool AiAssistantCoordinator::shouldUseAgentRuntime(const QString& resolvedDomainId) const
{
	return resolvedDomainId == AiDomainIds::meshCreate() || resolvedDomainId == AiDomainIds::meshCompose() ||
		   resolvedDomainId == AiDomainIds::pointCloudOps() || resolvedDomainId == AiDomainIds::documentImport() ||
		   resolvedDomainId == AiDomainIds::geometryOps() || resolvedDomainId == AiDomainIds::featureBuild() ||
		   resolvedDomainId == AiDomainIds::labelingAnnot() || resolvedDomainId == AiDomainIds::sceneOps() ||
		   resolvedDomainId == AiDomainIds::processFlow();
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
		if (ev.confirmKind == AiAgentConfirmKind::TrajectoryCommit)
		{
			const QByteArray plan =
				!ev.proposedArgsJson.isEmpty() && ev.proposedArgsJson != QByteArrayLiteral("{}")
					? ev.proposedArgsJson
					: m_pendingFeaturePlanJson;
			m_dock->appendSystemMessage(ev.message.isEmpty()
											? QStringLiteral("请在对话框中确认离散策略与管线算子。")
											: ev.message);
			openTrajectoryDiscretizeDialog(ev.pendingId, plan, !ev.secondaryLabel.isEmpty());
			break;
		}
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
		// 空 message：摘要已在 StepDone 贴过，避免「同一段贴两次」
		if (!ev.message.isEmpty())
			m_dock->appendAssistantMessage(ev.message);
		if (ev.toolId == QStringLiteral("geometry.recognize.create") ||
			ev.confirmKind == AiAgentConfirmKind::RecognizeCreate)
			m_pendingRecognitionJson.clear();
		if (ev.toolId == QStringLiteral("trajectory.feature.commit") ||
			ev.confirmKind == AiAgentConfirmKind::TrajectoryCommit)
		{
			const QString via = m_pendingFeatureParserVia;
			resetFeatureSession();
			emit assistantFinished(ev.message, false, via.isEmpty() ? QStringLiteral("Agent") : via);
			break;
		}
		emit assistantFinished(ev.message, false, QStringLiteral("Agent"));
		break;
	case AiAgentEventKind::Error:
		m_dock->hideAgentConfirmPanel();
		m_dock->setBusy(false);
		// 离散对话框取消：保留特征会话，不当作解析失败
		if (ev.message == QStringLiteral("已取消。") &&
			(m_featureSessionState == FeatureSessionState::PreviewCandidates ||
			 m_featureSessionState == FeatureSessionState::AwaitingSelection))
		{
			m_dock->appendSystemMessage(QStringLiteral("已取消离散确认，当前选择仍保留。"));
			break;
		}
		m_dock->appendAssistantMessage(ev.message);
		emit parseFailed(ev.message, QStringLiteral("Agent"));
		break;
	case AiAgentEventKind::Secondary:
		m_dock->hideAgentConfirmPanel();
		m_dock->setBusy(false);
		if (ev.toolId == QStringLiteral("trajectory.feature.commit"))
		{
			restoreTrajectoryCandidatePreview();
			m_dock->appendSystemMessage(
				QStringLiteral("已返回特征选择。可继续「选 N」调整，或再次输入「确认」打开离散对话框。"));
		}
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
