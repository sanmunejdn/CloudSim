/// @file AiAgentRuntime.cpp
/// @brief 状态机驱动的 Agent 循环

#include "Ai/AiAgentRuntime.h"

#include "Ai/AiAgentMemory.h"
#include "Ai/AiAgentPlanBuilder.h"
#include "Ai/AiAgentTrace.h"
#include "Ai/AiArgsSchema.h"
#include "Ai/AiAssistantHostImpl.h"
#include "Ai/AiCatalogKeywordMatcher.h"
#include "Ai/AiHostButtonApiDispatch.h"
#include "Ai/AiSceneSnapshotBuilder.h"
#include "AiDomainTypes.h"
#include "AiLlmClient.h"
#include "AiProgressSink.h"
#include "IPluginDocument.h"
#include "PluginHostContext.h"

#include <QUuid>
#include <QVariantMap>
#include <algorithm>

namespace
{
QString titleForApi(const nlohmann::json& api)
{
	if (api.contains("keywords") && api["keywords"].is_array() && !api["keywords"].empty() &&
		api["keywords"][0].is_string())
		return QString::fromStdString(api["keywords"][0].get<std::string>());
	return QString::fromStdString(api.value("id", "tool"));
}

nlohmann::json parseObj(const QByteArray& utf8)
{
	if (utf8.isEmpty())
		return nlohmann::json::object();
	try
	{
		return nlohmann::json::parse(utf8.constData(), nullptr, true);
	}
	catch (...)
	{
		return nlohmann::json::object();
	}
}

QString stateName(AiAgentState s)
{
	switch (s)
	{
	case AiAgentState::Idle:
		return QStringLiteral("Idle");
	case AiAgentState::Proposing:
		return QStringLiteral("Proposing");
	case AiAgentState::AwaitingConfirm:
		return QStringLiteral("AwaitingConfirm");
	case AiAgentState::Executing:
		return QStringLiteral("Executing");
	case AiAgentState::Done:
		return QStringLiteral("Done");
	}
	return QStringLiteral("?");
}

QStringList backendIdsInSnapshot(const QByteArray& snap)
{
	QStringList ids;
	const nlohmann::json j = parseObj(snap);
	if (!j.contains("objects") || !j["objects"].is_array())
		return ids;
	for (const auto& o : j["objects"])
	{
		if (o.is_object() && o.contains("id") && o["id"].is_string())
			ids << QString::fromStdString(o["id"].get<std::string>());
	}
	return ids;
}

QStringList newBackendIdsAfter(const QByteArray& beforeSnap, const QByteArray& afterSnap)
{
	const QStringList before = backendIdsInSnapshot(beforeSnap);
	QStringList added;
	for (const QString& id : backendIdsInSnapshot(afterSnap))
	{
		if (!before.contains(id))
			added << id;
	}
	return added;
}
} // namespace

AiAgentRuntime::AiAgentRuntime(PluginHostContext* host, AiAssistantHostImpl* assistant)
	: m_host(host), m_assistant(assistant)
{
}

void AiAgentRuntime::setState(AiAgentState s)
{
	m_state = s;
}

void AiAgentRuntime::trace(const QString& st, const QString& toolId, bool ok, const QByteArray& detail)
{
	if (!m_enableTrace || !m_host)
		return;
	AiAgentTrace::append(m_host->applicationDirPath(), st, toolId, ok, detail);
}

QString AiAgentRuntime::currentDocumentId() const
{
	if (!m_host || !m_host->activeDocument())
		return {};
	return QString::fromStdString(m_host->activeDocument()->documentId());
}

void AiAgentRuntime::runTurnAsync(const AiInferenceRequest& request, const AiConfigDto& config,
								  const AiInferenceProgressFn& progress, const AiAgentEventFn& onEvent)
{
	if (m_state != AiAgentState::Idle && m_state != AiAgentState::Done)
		cancelTurn();

	++m_generation;
	m_onEvent = onEvent;
	m_progress = progress;
	m_config = config;
	m_request = request;
	m_step = 0;
	m_doneTools.clear();
	m_pending.reset();
	m_plan.reset();
	m_planIndex = 0;
	m_planSummaryEmitted = false;
	m_didReplan = false;
	m_llmMessages = nlohmann::json::array();
	m_usedLlm = false;
	m_maxSteps = std::max(1, config.agent.maxSteps);
	m_planMaxSteps = std::max(1, config.agent.planMaxSteps);
	m_autoLowRisk = config.agent.autoExecuteLowRisk;
	m_requireKeywordHit = config.agent.requireKeywordHit;
	m_enableTrace = config.agent.enableTrace;
	m_enablePlan = config.agent.enablePlan;
	m_replanOnFailure = config.agent.replanOnFailure;
	m_catalog = m_assistant ? m_assistant->apiCatalogJson() : QByteArray();
	m_snapshot = m_host ? AiSceneSnapshotBuilder::buildJson(*m_host) : QByteArrayLiteral("{}");
	setState(AiAgentState::Proposing);
	trace(QStringLiteral("turn_start"), {}, true, m_request.userText.toUtf8());
	schedulePropose(false);
}

void AiAgentRuntime::beginDomainConfirm(const AiDomainConfirmRequest& request, const AiAgentEventFn& onEvent)
{
	if (m_state != AiAgentState::Idle && m_state != AiAgentState::Done)
		cancelTurn();

	++m_generation;
	m_onEvent = onEvent;
	m_progress = {};
	m_pending = std::make_unique<Pending>();
	m_pending->id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	m_pending->confirmKind = request.kind;
	m_pending->domainPayload = request.payloadUtf8;
	m_pending->title = request.title;
	m_pending->risk = request.risk;
	m_pending->confirmLabel = request.confirmLabel;
	m_pending->secondaryLabel = request.secondaryLabel;
	m_pending->schemaJson = QByteArrayLiteral("[]");
	// TrajectoryCommit：把特征计划放进 proposedArgs，供 Coordinator 弹离散对话框
	m_pending->proposedArgs =
		request.kind == AiAgentConfirmKind::TrajectoryCommit && !request.payloadUtf8.isEmpty()
			? request.payloadUtf8
			: QByteArrayLiteral("{}");
	m_pending->stepIndex = 0;
	if (request.kind == AiAgentConfirmKind::RecognizeCreate)
		m_pending->toolId = QStringLiteral("geometry.recognize.create");
	else
		m_pending->toolId = QStringLiteral("trajectory.feature.commit");
	m_snapshot = m_host ? AiSceneSnapshotBuilder::buildJson(*m_host) : QByteArrayLiteral("{}");
	setState(AiAgentState::AwaitingConfirm);
	trace(QStringLiteral("domain_confirm"), m_pending->toolId, true, request.payloadUtf8);
	emitNeedConfirm();
}

void AiAgentRuntime::schedulePropose(bool afterToolObservation)
{
	if (!m_host || m_state == AiAgentState::Idle || m_state == AiAgentState::Done)
		return;
	setState(AiAgentState::Proposing);
	const int gen = m_generation;
	m_host->enqueueJob(
		QStringLiteral("AI: agent propose"),
		[this, gen, afterToolObservation](const PluginJobProgressFn&)
		{
			if (gen != m_generation)
				return;
			QString via;
			if (!afterToolObservation && m_enablePlan && !m_plan.has_value())
				tryBuildInitialPlan(&via);

			QString toolId;
			QByteArray args;
			QString toolCallId;
			const bool ok = proposeNextTool(&toolId, &args, &via, &toolCallId);
			m_host->invokeOnUiThread(
				[this, gen, ok, toolId, args, via, toolCallId, afterToolObservation]()
				{
					if (gen != m_generation)
						return;
					if (m_state != AiAgentState::Proposing)
						return;
					if (!ok)
					{
						if (!m_doneTools.isEmpty())
							finishOk(via.isEmpty() ? QStringLiteral("已完成 %1 步。").arg(m_doneTools.size()) : via);
						else
							finishOk(via.isEmpty() ? QStringLiteral(
														 "未能识别可执行命令。可尝试按钮名，如「体素下采样」「点云匹配」。")
												   : via);
						return;
					}
					emitPlanSummaryIfNeeded();
					m_pending = std::make_unique<Pending>();
					m_pending->id = QUuid::createUuid().toString(QUuid::WithoutBraces);
					m_pending->toolId = toolId;
					m_pending->confirmKind = AiAgentConfirmKind::CatalogTool;
					m_pending->lastToolCallId = toolCallId;
					m_pending->stepIndex = m_step;
					const QByteArray entry = apiEntryJson(toolId);
					nlohmann::json api = parseObj(entry);
					m_pending->risk = QString::fromStdString(api.value("risk", "medium"));
					m_pending->title = titleForApi(api);
					if (m_plan && m_plan->steps.size() > 1)
					{
						m_pending->title =
							QStringLiteral("计划 %1/%2 · %3")
								.arg(m_planIndex + 1)
								.arg(m_plan->steps.size())
								.arg(m_pending->title);
					}
					if (api.contains("args_schema"))
						m_pending->schemaJson = QByteArray::fromStdString(api["args_schema"].dump());
					else
						m_pending->schemaJson = QByteArrayLiteral("[]");
					m_pending->proposedArgs = mergePrefs(toolId, args);
					(void)afterToolObservation;
					emitNeedConfirm();
				});
		},
		[this, gen](bool threw, const QString& throwMessage)
		{
			if (!threw || gen != m_generation)
				return;
			m_host->invokeOnUiThread(
				[this, gen, throwMessage]()
				{
					if (gen != m_generation)
						return;
					finishErr(throwMessage.isEmpty() ? QStringLiteral("Agent 提案失败。") : throwMessage);
				});
		});
}

void AiAgentRuntime::emitNeedConfirm()
{
	if (!m_pending)
		return;
	const bool schemaEmpty = parseObj(m_pending->schemaJson).empty();
	if (m_pending->confirmKind == AiAgentConfirmKind::CatalogTool && m_autoLowRisk && schemaEmpty &&
		m_pending->risk.compare(QStringLiteral("low"), Qt::CaseInsensitive) == 0)
	{
		continueAfterConfirm(QByteArrayLiteral("{}"));
		return;
	}

	setState(AiAgentState::AwaitingConfirm);
	if (!m_onEvent)
		return;
	AiAgentEvent ev;
	ev.kind = AiAgentEventKind::NeedConfirm;
	ev.pendingId = m_pending->id;
	ev.toolId = m_pending->toolId;
	ev.title = m_pending->title;
	ev.risk = m_pending->risk;
	ev.argsSchemaJson = m_pending->schemaJson;
	ev.proposedArgsJson = m_pending->proposedArgs;
	ev.sceneSnapshotJson = m_snapshot;
	ev.stepIndex = m_pending->stepIndex;
	ev.confirmLabel = m_pending->confirmLabel;
	ev.secondaryLabel = m_pending->secondaryLabel;
	ev.confirmKind = m_pending->confirmKind;
	ev.message = m_pending->confirmKind == AiAgentConfirmKind::TrajectoryCommit
					 ? QStringLiteral("请在对话框中确认离散策略与管线算子。")
					 : QStringLiteral("请确认参数后执行。");
	m_onEvent(ev);
}

void AiAgentRuntime::submitConfirm(const QString& pendingId, const QByteArray& argsJsonUtf8)
{
	if (m_state != AiAgentState::AwaitingConfirm || !m_pending || m_pending->id != pendingId)
		return;
	continueAfterConfirm(argsJsonUtf8);
}

void AiAgentRuntime::secondaryConfirm(const QString& pendingId)
{
	if (m_state != AiAgentState::AwaitingConfirm || !m_pending || m_pending->id != pendingId)
		return;
	const QString toolId = m_pending->toolId;
	m_pending.reset();
	setState(AiAgentState::Done);
	if (m_onEvent)
	{
		AiAgentEvent ev;
		ev.kind = AiAgentEventKind::Secondary;
		ev.toolId = toolId;
		ev.message = QStringLiteral("secondary");
		m_onEvent(ev);
	}
	setState(AiAgentState::Idle);
}

void AiAgentRuntime::cancelConfirm(const QString& pendingId)
{
	if (!m_pending || m_pending->id != pendingId)
		return;
	finishErr(QStringLiteral("已取消。"));
}

void AiAgentRuntime::cancelTurn()
{
	++m_generation;
	m_pending.reset();
	m_plan.reset();
	m_planIndex = 0;
	const bool wasActive = m_state != AiAgentState::Idle && m_state != AiAgentState::Done;
	setState(AiAgentState::Idle);
	if (wasActive && m_onEvent)
	{
		AiAgentEvent ev;
		ev.kind = AiAgentEventKind::Error;
		ev.message = QStringLiteral("已中断。");
		ev.isError = true;
		m_onEvent(ev);
	}
	m_onEvent = {};
	trace(QStringLiteral("cancel"), {}, true, {});
}

bool AiAgentRuntime::hasPlanRemaining() const
{
	return m_plan.has_value() && m_planIndex < m_plan->steps.size();
}

void AiAgentRuntime::emitPlanSummaryIfNeeded()
{
	if (m_planSummaryEmitted || !m_plan || m_plan->steps.size() <= 1 || !m_onEvent)
		return;
	m_planSummaryEmitted = true;
	AiAgentEvent ev;
	ev.kind = AiAgentEventKind::StepDone;
	ev.message = QStringLiteral("计划（%1 步）：%2").arg(m_plan->steps.size()).arg(m_plan->summary);
	ev.stepIndex = -1;
	m_onEvent(ev);
}

const AiDomainModelConfig* AiAgentRuntime::findDomainConfig(const QString& domainId) const
{
	for (const auto& d : m_config.domains)
	{
		if (d.id == domainId && d.enabled)
			return &d;
	}
	return nullptr;
}

bool AiAgentRuntime::tryBuildInitialPlan(QString* via)
{
	const QString domain = m_assistant->resolveDomainId(m_request.domainId, m_request.userText);
	AiAgentPlanBuilder::BuildInput in;
	in.userText = m_request.userText;
	in.domainId = domain;
	in.catalogJsonUtf8 = m_catalog;
	in.sceneSnapshotUtf8 = m_snapshot;
	in.sessionSummaryUtf8 = AiAgentMemory::sessionSummaryUtf8();
	in.maxSteps = std::min(m_maxSteps, m_planMaxSteps);
	// 无 keyword 策略下禁止 LLM 编造步骤；场景/工艺规则与多 keyword 仍可用
	in.enableLlmPlan = m_enablePlan && !m_requireKeywordHit;
	if (const AiDomainModelConfig* dm = findDomainConfig(domain))
	{
		in.llm.enabled = in.enableLlmPlan;
		in.llm.baseUrl = dm->baseUrl;
		in.llm.model = dm->model;
		in.llm.timeoutMs = 120000;
		in.llm.temperature = 0.1;
	}
	else
		in.enableLlmPlan = false;
	in.progress = [this](double f, const QString& m)
	{
		if (m_progress)
			m_progress(f, m);
	};

	AiAgentPlan plan = AiAgentPlanBuilder::buildPlan(in);
	if (plan.steps.isEmpty())
		return false;
	m_plan = std::move(plan);
	m_planIndex = 0;
	if (via)
		*via = QStringLiteral("Plan");
	trace(QStringLiteral("plan_built"), {}, true, m_plan->summary.toUtf8());
	return true;
}

bool AiAgentRuntime::tryReplanAfterFailure(const QString& failureObservation)
{
	if (!m_replanOnFailure || m_didReplan || !m_enablePlan)
		return false;
	const QString domain = m_assistant->resolveDomainId(m_request.domainId, m_request.userText);
	AiAgentPlanBuilder::BuildInput in;
	in.userText = m_request.userText;
	in.domainId = domain;
	in.catalogJsonUtf8 = m_catalog;
	in.sceneSnapshotUtf8 = m_snapshot;
	in.sessionSummaryUtf8 = AiAgentMemory::sessionSummaryUtf8();
	in.maxSteps = std::min(m_maxSteps - m_step, m_planMaxSteps);
	in.enableLlmPlan = m_enablePlan && !m_requireKeywordHit;
	if (const AiDomainModelConfig* dm = findDomainConfig(domain))
	{
		in.llm.enabled = in.enableLlmPlan;
		in.llm.baseUrl = dm->baseUrl;
		in.llm.model = dm->model;
		in.llm.timeoutMs = 120000;
		in.llm.temperature = 0.1;
	}
	else
		return false;
	in.progress = [this](double f, const QString& m)
	{
		if (m_progress)
			m_progress(f, m);
	};

	// 仅排除「一次建成」类 API；translate 等允许同 api 多段
	for (const QString& id : m_doneTools)
	{
		if (id == QStringLiteral("createPrimitiveMesh"))
			in.excludeApiIds.append(id);
	}
	AiAgentPlan plan = AiAgentPlanBuilder::buildPlan(in, failureObservation);
	if (plan.steps.isEmpty())
		return false;
	m_didReplan = true;
	m_plan = std::move(plan);
	m_planIndex = 0;
	m_planSummaryEmitted = false;
	trace(QStringLiteral("replan"), {}, true, m_plan->summary.toUtf8());
	if (m_onEvent)
	{
		AiAgentEvent ev;
		ev.kind = AiAgentEventKind::StepDone;
		ev.message = QStringLiteral("已重规划剩余步骤：%1").arg(m_plan->summary);
		m_onEvent(ev);
	}
	return true;
}

bool AiAgentRuntime::userWantsMultiStep() const
{
	const QString t = m_request.userText;
	// 「和/先」单独子串误伤面太大（如「生成」），多步靠 plan / keyword 串联
	return t.contains(QStringLiteral("然后")) || t.contains(QStringLiteral("再")) ||
		   t.contains(QStringLiteral("并且")) || t.contains(QStringLiteral("接着"));
}

bool AiAgentRuntime::hasRemainingKeywordMatch() const
{
	if (!m_assistant)
		return false;
	const QString domain = m_assistant->resolveDomainId(m_request.domainId, m_request.userText);
	return AiCatalogKeywordMatcher::tryMatch(m_catalog, m_request.userText, domain, m_doneTools).ok;
}

int AiAgentRuntime::objectCountInSnapshot(const QByteArray& snap) const
{
	const nlohmann::json j = parseObj(snap);
	if (!j.contains("objects") || !j["objects"].is_array())
		return 0;
	return static_cast<int>(j["objects"].size());
}

QString AiAgentRuntime::formatObservation(const QString& toolId, const QByteArray& argsJson,
										  const AiToolResult& result) const
{
	const nlohmann::json api = parseObj(apiEntryJson(toolId));
	const QString title = titleForApi(api);
	const nlohmann::json args = parseObj(argsJson);
	QStringList bits;
	bits << title;
	auto addArg = [&](const char* key)
	{
		if (!args.contains(key))
			return;
		const auto& v = args[key];
		if (v.is_string())
			bits << QStringLiteral("%1=%2").arg(QLatin1String(key), QString::fromStdString(v.get<std::string>()));
		else if (v.is_number())
			bits << QStringLiteral("%1=%2").arg(QLatin1String(key)).arg(v.get<double>());
	};
	addArg("backend_id");
	addArg("source_backend_id");
	addArg("target_backend_id");
	addArg("voxel_mm");
	addArg("ratio");
	addArg("radius_mm");
	addArg("path");
	addArg("dx_mm");
	addArg("dy_mm");
	addArg("dz_mm");
	addArg("rx_deg");
	addArg("ry_deg");
	addArg("rz_deg");
	const int after = objectCountInSnapshot(m_snapshot);
	if (m_objectsBeforeStep != after)
		bits << QStringLiteral("对象数 %1→%2").arg(m_objectsBeforeStep).arg(after);
	if (!result.newBackendIds.isEmpty())
		bits << QStringLiteral("new=%1").arg(result.newBackendIds.join(QLatin1Char(',')));
	QString msg = bits.join(QStringLiteral(" · "));
	if (!result.summary.isEmpty() && result.summary != QStringLiteral("已执行"))
		msg += QStringLiteral("\n") + result.summary;
	return msg;
}

AiToolResult AiAgentRuntime::executeCatalogTool(const QString& toolId, const nlohmann::json& args)
{
	AiToolResult r = AiHostButtonApiDispatch::execute(*m_host, toolId.toStdString(), args, false);
	if (r.handled)
		return r;

	nlohmann::json plan;
	plan["version"] = 2;
	nlohmann::json step;
	step["id"] = "s1";
	step["api"] = toolId.toStdString();
	step["args"] = args;
	plan["steps"] = nlohmann::json::array({step});
	nlohmann::json execPlan = plan;
	if (toolId == QStringLiteral("createPrimitiveMesh"))
	{
		nlohmann::json cmd;
		cmd["version"] = 1;
		cmd["action"] = "create_mesh";
		cmd["primitive"] = args.value("primitive", "box");
		nlohmann::json dim = nlohmann::json::object();
		if (args.contains("length_mm"))
			dim["length"] = args["length_mm"];
		if (args.contains("width_mm"))
			dim["width"] = args["width_mm"];
		if (args.contains("height_mm"))
			dim["height"] = args["height_mm"];
		if (args.contains("radius_mm"))
			dim["radius"] = args["radius_mm"];
		if (!dim.empty())
			cmd["dimensions_mm"] = dim;
		execPlan = cmd;
	}
	QString summary;
	QString err;
	r.handled = true;
	r.ok = m_assistant && m_assistant->executeActionPlan(QByteArray::fromStdString(execPlan.dump()), &summary, &err);
	r.summary = summary;
	r.error = err;
	return r;
}

void AiAgentRuntime::continueAfterConfirm(const QByteArray& argsJson)
{
	if (!m_pending || !m_host)
	{
		finishErr(QStringLiteral("无待确认步骤。"));
		return;
	}
	setState(AiAgentState::Executing);
	const auto kind = m_pending->confirmKind;
	const QString toolId = m_pending->toolId;
	const QByteArray domainPayload = m_pending->domainPayload;
	const QString toolCallId = m_pending->lastToolCallId;

	AiToolResult result;
	const QByteArray snapBefore = m_snapshot;
	m_objectsBeforeStep = objectCountInSnapshot(m_snapshot);

	if (kind == AiAgentConfirmKind::RecognizeCreate)
	{
		QString summary;
		QString err;
		result.handled = true;
		result.ok = m_assistant && m_assistant->executeDomainOutput(AiDomainIds::geometryRecognize(), domainPayload,
																   &summary, &err);
		result.summary = summary;
		result.error = err;
	}
	else if (kind == AiAgentConfirmKind::TrajectoryCommit)
	{
		QString summary;
		QString err;
		result.handled = true;
		// 对话框 Accept 后传入 merged 计划；无则回退 beginDomainConfirm 时的 payload
		const QByteArray commitPayload = argsJson.trimmed().isEmpty() || argsJson == QByteArrayLiteral("{}")
											 ? domainPayload
											 : argsJson;
		result.ok = m_host->commitAiTrajectoryFeatures(commitPayload, &summary, &err);
		result.summary = summary;
		result.error = err;
	}
	else
	{
		const nlohmann::json args = parseObj(argsJson);
		QString missErr;
		const nlohmann::json api = parseObj(apiEntryJson(toolId));
		if (AiArgsSchema::missingRequiredArgs(api.value("args_schema", nlohmann::json::array()), args, &missErr))
		{
			result.handled = true;
			result.ok = false;
			result.error = missErr;
		}
		else
		{
			result = executeCatalogTool(toolId, args);
		}
	}

	if (!result.ok)
	{
		AiAgentMemory::appendSessionStep(toolId, false, result.error);
		trace(QStringLiteral("execute_fail"), toolId, false, result.error.toUtf8());
		const QString errMsg = result.error.isEmpty() ? QStringLiteral("执行失败。") : result.error;
		m_pending.reset();
		++m_step;
		if (kind == AiAgentConfirmKind::CatalogTool && m_replanOnFailure && !m_didReplan && m_enablePlan)
		{
			const int gen = m_generation;
			setState(AiAgentState::Proposing);
			m_host->enqueueJob(
				QStringLiteral("AI: agent replan"),
				[this, gen, errMsg](const PluginJobProgressFn&)
				{
					if (gen != m_generation)
						return;
					const bool ok = tryReplanAfterFailure(errMsg);
					m_host->invokeOnUiThread(
						[this, gen, ok, errMsg]()
						{
							if (gen != m_generation)
								return;
							if (ok)
								schedulePropose(true);
							else
								finishErr(errMsg);
						});
				},
				[this, gen, errMsg](bool threw, const QString&)
				{
					if (!threw || gen != m_generation)
						return;
					m_host->invokeOnUiThread(
						[this, gen, errMsg]()
						{
							if (gen != m_generation)
								return;
							finishErr(errMsg);
						});
				});
			return;
		}
		finishErr(errMsg);
		return;
	}

	if (kind == AiAgentConfirmKind::CatalogTool)
	{
		nlohmann::json args = parseObj(argsJson);
		QVariantMap prefs;
		for (auto it = args.begin(); it != args.end(); ++it)
		{
			if (it.value().is_string())
				prefs.insert(QString::fromStdString(it.key()), QString::fromStdString(it.value().get<std::string>()));
			else if (it.value().is_number_float() || it.value().is_number_integer())
				prefs.insert(QString::fromStdString(it.key()), it.value().get<double>());
			else if (it.value().is_boolean())
				prefs.insert(QString::fromStdString(it.key()), it.value().get<bool>());
		}
		AiAgentMemory::savePrefForApi(m_host->applicationDirPath(), toolId, prefs, currentDocumentId());
	}

	m_snapshot = AiSceneSnapshotBuilder::buildJson(*m_host);
	if (result.newBackendIds.isEmpty())
		result.newBackendIds = newBackendIdsAfter(snapBefore, m_snapshot);
	const QString observation =
		kind == AiAgentConfirmKind::CatalogTool
			? formatObservation(toolId, argsJson, result)
			: (result.summary.isEmpty() ? QStringLiteral("已确认执行") : result.summary);
	AiAgentMemory::appendSessionStep(toolId, true, observation);
	m_doneTools.append(toolId);
	if (m_plan.has_value())
		++m_planIndex;
	trace(QStringLiteral("execute_ok"), toolId, true, observation.toUtf8());

	if (m_usedLlm && !m_llmMessages.empty())
		AiLlmClient::appendToolObservation(m_llmMessages, toolCallId, toolId, observation.toUtf8());

	if (m_onEvent)
	{
		AiAgentEvent ev;
		ev.kind = AiAgentEventKind::StepDone;
		ev.toolId = toolId;
		ev.message = observation;
		ev.stepIndex = m_step;
		m_onEvent(ev);
	}

	m_pending.reset();
	++m_step;

	if (kind != AiAgentConfirmKind::CatalogTool)
	{
		finishOk(observation, toolId, kind);
		return;
	}

	// 单次 LLM tool_calls 成功后不要因 m_usedLlm 再提案：模型常会再调一次 create，
	// 失败后重规划又会再建实体（「生成长方体」→ 两个长方体）。
	const bool more =
		m_step < m_maxSteps && (hasPlanRemaining() || hasRemainingKeywordMatch() || userWantsMultiStep());
	if (!more)
	{
		finishOk(observation, toolId, kind);
		return;
	}
	schedulePropose(true);
}

bool AiAgentRuntime::proposeNextTool(QString* toolId, QByteArray* argsJson, QString* via, QString* toolCallId)
{
	if (hasPlanRemaining())
	{
		const AiAgentPlanStep& s = m_plan->steps.at(m_planIndex);
		*toolId = s.apiId;
		*argsJson = s.argsJson;
		*via = s.rationale.isEmpty() ? QStringLiteral("Plan") : s.rationale;
		if (toolCallId)
			toolCallId->clear();
		return true;
	}

	const QString domain = m_assistant->resolveDomainId(m_request.domainId, m_request.userText);
	const auto match = AiCatalogKeywordMatcher::tryMatch(m_catalog, m_request.userText, domain, m_doneTools);
	if (match.ok)
	{
		*toolId = match.apiId;
		nlohmann::json plan = parseObj(match.planJsonUtf8);
		nlohmann::json args = nlohmann::json::object();
		if (plan.contains("steps") && plan["steps"].is_array() && !plan["steps"].empty())
			args = plan["steps"][0].value("args", nlohmann::json::object());
		*argsJson = QByteArray::fromStdString(args.dump());
		*via = match.hintMessage.isEmpty() ? QStringLiteral("Rules") : match.hintMessage;
		if (toolCallId)
			toolCallId->clear();
		return true;
	}

	// 无可靠 keyword：不猜工具；场景/工艺规则规划仍可走 hasPlanRemaining
	if (m_requireKeywordHit)
	{
		if (via)
		{
			*via = QStringLiteral(
				"未匹配到可靠的按钮关键词，未自动调用工具。\n"
				"请改用 Dock 按钮原文（如「体素下采样」「点云匹配」），或先选择领域后再描述。");
		}
		return false;
	}

	const AiDomainModelConfig* dm = findDomainConfig(domain);
	if (!dm)
		return false;

	AiLlmConfig llm;
	llm.enabled = true;
	llm.baseUrl = dm->baseUrl;
	llm.model = dm->model;
	llm.timeoutMs = 120000;
	llm.temperature = 0.1;
	const QByteArray tools = AiArgsSchema::buildOpenAiToolsFromCatalog(m_catalog, domain, m_doneTools);
	const AiProgressSink sink = [this](double f, const QString& m)
	{
		if (m_progress)
			m_progress(f, m);
	};
	m_usedLlm = true;
	const auto lr = AiLlmClient::chatWithTools(m_request.userText, llm, sink, tools, m_snapshot,
											   AiAgentMemory::sessionSummaryUtf8(), &m_llmMessages);
	if (!lr.ok)
	{
		if (via)
			*via = lr.errorMessage;
		return false;
	}
	if (!lr.hasToolCall)
	{
		if (via)
			*via = lr.assistantText.isEmpty() ? QStringLiteral("本轮结束。") : lr.assistantText;
		return false;
	}
	if (m_doneTools.contains(lr.toolName))
		return false;
	*toolId = lr.toolName;
	*argsJson = QByteArray::fromStdString(lr.args.dump());
	*via = QStringLiteral("LLM tool_calls");
	if (toolCallId)
		*toolCallId = lr.toolCallId;
	return true;
}

QByteArray AiAgentRuntime::apiEntryJson(const QString& toolId) const
{
	nlohmann::json root = parseObj(m_catalog);
	if (!root.contains("apis") || !root["apis"].is_array())
		return QByteArrayLiteral("{}");
	for (const auto& api : root["apis"])
	{
		if (api.is_object() && api.value("id", "") == toolId.toStdString())
			return QByteArray::fromStdString(api.dump());
	}
	return QByteArrayLiteral("{}");
}

QByteArray AiAgentRuntime::mergePrefs(const QString& toolId, const QByteArray& proposed) const
{
	nlohmann::json args = parseObj(proposed);
	const QVariantMap prefs = AiAgentMemory::loadPrefs(m_host->applicationDirPath(), currentDocumentId());
	const QVariant pref = prefs.value(toolId);
	if (pref.canConvert<QVariantMap>())
	{
		const QVariantMap m = pref.toMap();
		for (auto it = m.begin(); it != m.end(); ++it)
		{
			const std::string k = it.key().toStdString();
			if (args.contains(k) && !args[k].is_null())
				continue;
			if (it.value().type() == QVariant::Bool)
				args[k] = it.value().toBool();
			else if (it.value().type() == QVariant::Double || it.value().type() == QVariant::Int ||
					 it.value().type() == QVariant::LongLong)
				args[k] = it.value().toDouble();
			else
				args[k] = it.value().toString().toStdString();
		}
	}
	const QString sel = m_host->selectedBackendId();
	if (!sel.isEmpty())
	{
		if (!args.contains("backend_id") || !args["backend_id"].is_string() ||
			args["backend_id"].get<std::string>().empty())
			args["backend_id"] = sel.toStdString();
	}
	return QByteArray::fromStdString(args.dump());
}

void AiAgentRuntime::finishOk(const QString& message, const QString& toolId, AiAgentConfirmKind kind)
{
	m_pending.reset();
	setState(AiAgentState::Done);
	trace(QStringLiteral("finished"), toolId, true, message.toUtf8());
	if (m_onEvent)
	{
		AiAgentEvent ev;
		ev.kind = AiAgentEventKind::Finished;
		ev.message = message;
		ev.toolId = toolId;
		ev.confirmKind = kind;
		m_onEvent(ev);
	}
	setState(AiAgentState::Idle);
	m_onEvent = {};
}

void AiAgentRuntime::finishErr(const QString& message)
{
	m_pending.reset();
	setState(AiAgentState::Done);
	trace(QStringLiteral("error"), {}, false, message.toUtf8());
	if (m_onEvent)
	{
		AiAgentEvent ev;
		ev.kind = AiAgentEventKind::Error;
		ev.message = message;
		ev.isError = true;
		m_onEvent(ev);
	}
	setState(AiAgentState::Idle);
	m_onEvent = {};
}
